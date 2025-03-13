use alloc::boxed::Box;
use core::future;
use core::mem::MaybeUninit;
use core::task::Poll;

use async_trait::async_trait;
use smoltcp::socket::udp;
use smoltcp::socket::udp::UdpMetadata;
use smoltcp::wire::IpEndpoint;

use crate::executor::block_on;
use crate::executor::network::{Handle, NIC};
use crate::fd::{self, Endpoint, ListenEndpoint, ObjectInterface, PollEvent};
use crate::io;

#[derive(Debug)]
pub struct Socket {
	handle: Handle,
	nonblocking: bool,
	endpoint: Option<IpEndpoint>,
}

impl Socket {
	pub fn new(handle: Handle) -> Self {
		Self {
			handle,
			nonblocking: false,
			endpoint: None,
		}
	}

	fn with<R>(&self, f: impl FnOnce(&mut udp::Socket<'_>) -> R) -> R {
		let mut guard = NIC.lock();
		let nic = guard.as_nic_mut().unwrap();
		f(nic.get_mut_socket::<udp::Socket<'_>>(self.handle))
	}

	async fn close(&self) -> io::Result<()> {
		future::poll_fn(|_cx| {
			self.with(|socket| {
				socket.close();
				Poll::Ready(Ok(()))
			})
		})
		.await
	}

	async fn write_with_meta(&self, buffer: &[u8], meta: &UdpMetadata) -> io::Result<usize> {
		future::poll_fn(|cx| {
			self.with(|socket| {
				if socket.is_open() {
					if socket.can_send() {
						Poll::Ready(
							socket
								.send_slice(buffer, *meta)
								.map(|()| buffer.len())
								.map_err(|_| io::Error::EIO),
						)
					} else {
						socket.register_recv_waker(cx.waker());
						Poll::Pending
					}
				} else {
					Poll::Ready(Err(io::Error::EIO))
				}
			})
		})
		.await
	}

	async fn poll(&self, event: PollEvent) -> io::Result<PollEvent> {
		future::poll_fn(|cx| {
			self.with(|socket| {
				let ret = if socket.is_open() {
					let mut avail = PollEvent::empty();

					if socket.can_send() {
						avail.insert(
							PollEvent::POLLOUT | PollEvent::POLLWRNORM | PollEvent::POLLWRBAND,
						);
					}

					if socket.can_recv() {
						avail.insert(
							PollEvent::POLLIN | PollEvent::POLLRDNORM | PollEvent::POLLRDBAND,
						);
					}

					event & avail
				} else {
					PollEvent::POLLNVAL
				};

				if ret.is_empty() {
					if event.intersects(
						PollEvent::POLLIN | PollEvent::POLLRDNORM | PollEvent::POLLRDBAND,
					) {
						socket.register_recv_waker(cx.waker());
					}

					if event.intersects(
						PollEvent::POLLOUT | PollEvent::POLLWRNORM | PollEvent::POLLWRBAND,
					) {
						socket.register_send_waker(cx.waker());
					}

					Poll::Pending
				} else {
					Poll::Ready(Ok(ret))
				}
			})
		})
		.await
	}

	async fn bind(&self, endpoint: ListenEndpoint) -> io::Result<()> {
		#[allow(irrefutable_let_patterns)]
		if let ListenEndpoint::Ip(endpoint) = endpoint {
			self.with(|socket| socket.bind(endpoint).map_err(|_| io::Error::EADDRINUSE))
		} else {
			Err(io::Error::EIO)
		}
	}

	async fn connect(&mut self, endpoint: Endpoint) -> io::Result<()> {
		#[allow(irrefutable_let_patterns)]
		if let Endpoint::Ip(endpoint) = endpoint {
			self.endpoint = Some(endpoint);
			Ok(())
		} else {
			Err(io::Error::EIO)
		}
	}

	async fn sendto(&self, buf: &[u8], endpoint: Endpoint) -> io::Result<usize> {
		#[allow(irrefutable_let_patterns)]
		if let Endpoint::Ip(endpoint) = endpoint {
			let meta = UdpMetadata::from(endpoint);
			self.write_with_meta(buf, &meta).await
		} else {
			Err(io::Error::EIO)
		}
	}

	async fn recvfrom(&self, buffer: &mut [MaybeUninit<u8>]) -> io::Result<(usize, Endpoint)> {
		future::poll_fn(|cx| {
			self.with(|socket| {
				if socket.is_open() {
					if socket.can_recv() {
						match socket.recv() {
							// Drop the packet when the provided buffer cannot
							// fit the payload.
							Ok((data, meta)) if data.len() <= buffer.len() => {
								if self.endpoint.is_none_or(|ep| meta.endpoint == ep) {
									buffer[..data.len()].write_copy_of_slice(data);
									Poll::Ready(Ok((data.len(), meta.endpoint)))
								} else {
									socket.register_recv_waker(cx.waker());
									Poll::Pending
								}
							}
							_ => Poll::Ready(Err(io::Error::EIO)),
						}
					} else {
						socket.register_recv_waker(cx.waker());
						Poll::Pending
					}
				} else {
					Poll::Ready(Err(io::Error::EIO))
				}
			})
		})
		.await
		.map(|(len, endpoint)| (len, Endpoint::Ip(endpoint)))
	}

	async fn read(&self, buffer: &mut [MaybeUninit<u8>]) -> io::Result<usize> {
		future::poll_fn(|cx| {
			self.with(|socket| {
				if socket.is_open() {
					if socket.can_recv() {
						match socket.recv() {
							// Drop the packet when the provided buffer cannot
							// fit the payload.
							Ok((data, meta)) if data.len() <= buffer.len() => {
								if self.endpoint.is_none_or(|ep| meta.endpoint == ep) {
									buffer[..data.len()].write_copy_of_slice(data);
									Poll::Ready(Ok(data.len()))
								} else {
									socket.register_recv_waker(cx.waker());
									Poll::Pending
								}
							}
							_ => Poll::Ready(Err(io::Error::EIO)),
						}
					} else {
						socket.register_recv_waker(cx.waker());
						Poll::Pending
					}
				} else {
					Poll::Ready(Err(io::Error::EIO))
				}
			})
		})
		.await
	}

	async fn write(&self, buf: &[u8]) -> io::Result<usize> {
		if let Some(endpoint) = self.endpoint {
			let meta = UdpMetadata::from(endpoint);
			self.write_with_meta(buf, &meta).await
		} else {
			Err(io::Error::EINVAL)
		}
	}

	async fn status_flags(&self) -> io::Result<fd::StatusFlags> {
		let status_flags = if self.nonblocking {
			fd::StatusFlags::O_NONBLOCK
		} else {
			fd::StatusFlags::empty()
		};

		Ok(status_flags)
	}

	async fn set_status_flags(&mut self, status_flags: fd::StatusFlags) -> io::Result<()> {
		self.nonblocking = status_flags.contains(fd::StatusFlags::O_NONBLOCK);
		Ok(())
	}
}

impl Drop for Socket {
	fn drop(&mut self) {
		let _ = block_on(self.close(), None);
		NIC.lock().as_nic_mut().unwrap().destroy_socket(self.handle);
	}
}

#[async_trait]
impl ObjectInterface for async_lock::RwLock<Socket> {
	async fn poll(&self, event: PollEvent) -> io::Result<PollEvent> {
		self.read().await.poll(event).await
	}

	async fn bind(&self, endpoint: ListenEndpoint) -> io::Result<()> {
		self.read().await.bind(endpoint).await
	}

	async fn connect(&self, endpoint: Endpoint) -> io::Result<()> {
		self.write().await.connect(endpoint).await
	}

	async fn sendto(&self, buffer: &[u8], endpoint: Endpoint) -> io::Result<usize> {
		self.read().await.sendto(buffer, endpoint).await
	}

	async fn recvfrom(&self, buffer: &mut [MaybeUninit<u8>]) -> io::Result<(usize, Endpoint)> {
		self.read().await.recvfrom(buffer).await
	}

	async fn read(&self, buffer: &mut [MaybeUninit<u8>]) -> io::Result<usize> {
		self.read().await.read(buffer).await
	}

	async fn write(&self, buf: &[u8]) -> io::Result<usize> {
		self.read().await.write(buf).await
	}

	async fn status_flags(&self) -> io::Result<fd::StatusFlags> {
		self.read().await.status_flags().await
	}

	async fn set_status_flags(&self, status_flags: fd::StatusFlags) -> io::Result<()> {
		self.write().await.set_status_flags(status_flags).await
	}
}
