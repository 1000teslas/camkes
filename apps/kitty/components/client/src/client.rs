#![no_std]
#![feature(lang_items)]
#![feature(collections)]
#![feature(alloc)]
extern crate sel4_start;
extern crate sel4_sys;
extern crate std;
#[macro_use]
extern crate collections;
extern crate allocator_libc;
use core::fmt;
use core::sync::atomic::AtomicBool;
use collections::vec::Vec;
extern crate alloc;
use alloc::boxed::Box;

#[allow(dead_code, non_camel_case_types, non_snake_case)]
mod socket;
#[allow(dead_code, non_camel_case_types, non_snake_case)]
mod bindings;
#[allow(dead_code, non_camel_case_types, non_snake_case)]
mod stdio;

macro_rules! scan {
    ( $slice:expr, $sep:expr, $( $x:ty ),+ ) => {{
        let mut iter = $slice.split($sep);
        ($(iter.next().and_then(|word| word.parse::<$x>().ok()),)*)
    }}
}

const PF_INET: isize =   2;

static lock1: AtomicBool = core::sync::atomic::ATOMIC_BOOL_INIT;

fn spinlock_lock(lock: &AtomicBool) {
	while !lock.compare_and_swap(false, true, core::sync::atomic::Ordering::Relaxed) {

    }
}

fn spinlock_unlock(lock: &AtomicBool) {
	while !lock.compare_and_swap(true, false, core::sync::atomic::Ordering::Relaxed) {

    }
}


macro_rules! dprintf {
    ($($args:tt)*) => {
        {
            use ::core::fmt::Write;
            let mut dw = $crate::DebugWriter;
            write!(&mut dw, $($args)*).unwrap();
        }
    }
}

struct DebugWriter;

impl fmt::Write for DebugWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.as_bytes() {
            unsafe { sel4_sys::seL4_DebugPutChar(*b) };
        }
        Ok(())
    }
}

extern "C" {
    fn _run(a: isize) -> isize;
    pub fn readDataFromNetwork(bufferData: &[i8]);

}



fn kitty_server_communication_init() -> Result<(isize), isize> {
    let sockfd =
        unsafe {socket::socket(PF_INET, socket::Enum___socket_type::SOCK_STREAM as u32 as isize, 0)};
    if sockfd < 0 {
        dprintf!("\n Error : Could not create socket \n");
        return Err(0);
    }

    let mut serv_addr: socket::Struct_sockaddr_in = core::default::Default::default();
    serv_addr.sin_family = PF_INET as u16;
    serv_addr.sin_port = unsafe {socket::htons(3737)};
    serv_addr.sin_addr.s_addr = unsafe {socket::inet_addr(core::mem::transmute(b"10.13.0.1".as_ptr()))};

    let res = unsafe {socket::connect(sockfd,
        core::mem::transmute(&serv_addr),
        core::mem::size_of::<socket::Struct_sockaddr_in>())
    };
    if res < 0 {
        dprintf!("Error : connect failed\n");
        return Err(0);
    } else {
        dprintf!("Connect succeeded\n")
    }


    let mut recvBuff:[i8; 1024] = [0;1024];

    spinlock_lock(&lock1);
    let n = unsafe {
        stdio::read(sockfd,
            core::mem::transmute(&mut recvBuff[..].as_mut_ptr()),
            core::mem::size_of_val(&recvBuff) - 1)
    };
    spinlock_unlock(&lock1);

    if n <= 0 {
		dprintf!("Error: \n");
		return Err(0);
	}
    else
    {
        unsafe {readDataFromNetwork(&recvBuff[..])};


    }

    Ok(sockfd)
}

// fn is_whitespace(c:u8)-> bool {
//     return c == b" "
// }
//
// fn readDataFromNetwork(bufferData: &[u8]) {
//     let (cmd, payload) = scan!(bufferData, is_whitespace, usize, str);
//    // char *payload = malloc(sizeof(char)*1024);
//    // sscanf(bufferData, "%d %s", &cmd, payload);
//    //
//    // // Do processing of data
//    // intf("cmd: %d buff: %s\n", cmd, payload);
//    // if (cmd == RESP_SEED_TXID) {
//    //  	  //printf("cmd was RESP_SEED_TXID\n");
//    //  	  sscanf(payload,"%d",&txid);
//    // } else if (cmd == RESP_DISPLAY_MSG) {
//    // 	//printf("cmd was RESP_DISPLAY_MSG\n");
//    // 	display_clearText(512,380,1);
//    //  display_text(bufferData, 512, 380, 0,0,0);
//    // } else if (cmd == RESP_ALL_GOOD) {
//    // 	//printf("cmd was RESP_ALL_GOOD\n");
//    // 	beeps_beep();
//    //
//    // 	display_clearText(512,350,1);
//    //  display_text(bufferData, 512, 350, 0,0,0);
//    // } else if (cmd == RESP_NO_ACCOUNT) {
//    //    //printf("cmd was RESP_NO_ACCOUNT\n");
//    // 	display_clearText(512,350,1);
//    // eps_beep();
//    //  display_text(bufferData, 512, 350, 0,0,0);
//    // } else if (cmd == RESP_UNKNOWN_ERR) {
//    // 	//printf("cmd was RESP_UNKNOWN_ERR\n");
//    // 	display_clearText(512,350,1);
//    //  display_text(bufferData, 512, 350, 0,0,0);
//    // } else if (cmd ==RESP_ACC_DISABLED) {
//    // 	//printf("cmd was RESP_ACC_DISABLED\n");
//    // 	display_clearText(512,350,1);
//    //  display_text(bufferData, 512, 350, 0,0,0);
//    // }
//    //
//    //
//    // free(payload);
// }


#[no_mangle]
pub extern "C" fn run() -> isize {
    dprintf!("Kitty starts ... \n");
    dprintf!("About to init ... \n");
    unsafe {bindings::beeps_beep()};

    let a: Box<isize> = Box::new(4);
    dprintf!("{:?}\n", a);



    let fd = match kitty_server_communication_init() {
        Err(e) => panic!("uhoh{:?}", 0),
        Ok(a) => {a}
    };

    //
     unsafe { _run(fd) }
}

#[no_mangle]
pub extern "C" fn main12() {
    dprintf!("Hello, worl1d! I've changed!\n");
    //unsafe {test43()};
}

#[no_mangle]
#[allow(unused_variables)]
pub unsafe extern "C" fn __mulodi4(a: i64, b: i64, overflow: *mut i32) -> i64
{
    return a * b;
}

#[lang = "eh_unwind_resume"]
pub extern fn eh_unwind_resume(a:*mut u8) -> ! {
     loop{}

}
