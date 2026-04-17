use std::{
    ffi::CString,
    mem::size_of,
    os::fd::RawFd,
    pin::Pin,
    task::{Context, Poll},
};

use futures::stream::Stream;
use nix::libc;
use tokio::io::unix::AsyncFd;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WatchId(usize);

struct Watch {
    wd: i32,
    path: Box<str>,
}

pub struct InotifyStream {
    fd: RawFd,
    afd: AsyncFd<RawFd>,

    buf: [u8; 4096],
    len: usize,
    off: usize,

    watches: Vec<Watch>,
}

impl InotifyStream {
    pub fn new() -> Self {
        let fd = unsafe { libc::inotify_init1(libc::IN_NONBLOCK) };

        Self {
            fd,
            afd: AsyncFd::new(fd).unwrap(),
            buf: [0; 4096],
            len: 0,
            off: 0,
            watches: Vec::new(),
        }
    }

    pub fn watch(&mut self, path: &str) -> WatchId {
        let id = WatchId(self.watches.len());

        self.watches.push(Watch {
            wd: -1,
            path: path.into(),
        });

        self.rearm_watch(id);

        id
    }

    fn rearm_watch(&mut self, id: WatchId) {
        let w = &mut self.watches[id.0];
        let c = CString::new(&*w.path).unwrap();

        let mut attempts = 0;

        while attempts < 3 {
            let wd = unsafe {
                libc::inotify_add_watch(
                    self.fd,
                    c.as_ptr(),
                    libc::IN_CLOSE_WRITE
                        | libc::IN_MOVED_TO
                        | libc::IN_DELETE_SELF
                        | libc::IN_MOVE_SELF,
                )
            };

            if wd >= 0 {
                w.wd = wd;
                return;
            }

            attempts += 1;
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        w.wd = -1;
    }

    fn find_watch_mut_index(&self, wd: i32) -> Option<(WatchId, usize)> {
        self.watches
            .iter()
            .enumerate()
            .find(|(_, w)| w.wd == wd)
            .map(|(i, _)| (WatchId(i), i))
    }
}

impl Stream for InotifyStream {
    type Item = WatchId;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();

        loop {
            if this.off < this.len {
                let ev =
                    unsafe { &*(this.buf.as_ptr().add(this.off) as *const libc::inotify_event) };

                // this.off += size_of::<libc::inotify_event>() + ev.len as usize;

                let event_size = size_of::<libc::inotify_event>();
                let total = event_size + ev.len as usize;

                this.off += (total + std::mem::align_of::<libc::inotify_event>() - 1)
                    & !(std::mem::align_of::<libc::inotify_event>() - 1);

                if let Some((id, idx)) = this.find_watch_mut_index(ev.wd) {
                    let m = ev.mask;

                    if m & libc::IN_CLOSE_WRITE != 0 {
                        return Poll::Ready(Some(id));
                    }

                    if m & (libc::IN_MOVE_SELF | libc::IN_DELETE_SELF) != 0 {
                        this.rearm_watch(WatchId(idx));
                        return Poll::Ready(Some(id));
                    }
                }

                continue;
            }

            let mut guard = match this.afd.poll_read_ready(cx) {
                Poll::Ready(Ok(g)) => g,
                Poll::Pending => return Poll::Pending,
                Poll::Ready(Err(_)) => return Poll::Ready(None),
            };

            let len =
                unsafe { libc::read(this.fd, this.buf.as_mut_ptr() as *mut _, this.buf.len()) };

            if len < 0 {
                let err = std::io::Error::last_os_error();
                match err.raw_os_error() {
                    Some(libc::EAGAIN | libc::EINTR) => {
                        guard.clear_ready();
                        return Poll::Pending;
                    }
                    _ => {
                        guard.clear_ready();
                        return Poll::Ready(None);
                    }
                }
            }

            if len == 0 {
                guard.clear_ready();
                return Poll::Ready(None);
            }

            this.len = len as usize;
            this.off = 0;

            guard.clear_ready();
        }
    }
}
