import io
import serial
from serial.tools.list_ports_common import ListPortInfo 
from serial.tools.list_ports import comports
import time
from xmodem import XMODEM
import argparse
import zlib

ser: serial.Serial | None = None
def getc(size, timeout=1):
    global ser
    ret = ser.read(size) or None
    #if ret is not None:
    #    print(f"RX {len(ret)}B: {ret.hex()}")
    return ret

def putc(data, timeout=1):
    global ser
    #print(f"TX {len(data)}B: {data.hex()}")
    return ser.write(data)  # note that this ignores the timeout


def main():
    global ser
    # make a parser that accepts -p as a (COM) port argument and -b as baud, then accepts different subcommands
    # "flash" for flashing, "read" for reading/dumping memory, "reg" for register state 
    parser = argparse.ArgumentParser(description='EZ80 Firmware Uploader')
    parser.add_argument('-p', '--port', type=str, help='COM port to use (e.g. COM3)', default=None)
    parser.add_argument('-b', '--baud', type=int, help='Baud rate to use (default: 921600)', default=921600)

    subparsers = parser.add_subparsers(description='subcommands', dest='subparser', required=True)
    parser_flash = subparsers.add_parser('flash', help='Flash firmware')
    # reset after upload if "-r" flag is given
    parser_flash.add_argument('-r', "--reset", action='store_true', help='Reset device after flashing')
    parser_flash.add_argument('file', type=str, help='Firmware BIN file to upload')
    parsers_read = subparsers.add_parser('read', help='Read memory')
    # has start address (hex) and length (decimal) arguments
    parsers_read.add_argument('start', type=lambda x: int(x,0), help='Start address (hex)')
    parsers_read.add_argument('length', type=int, help='Length to read (decimal)')
    # reg subcommand exists with no arguments
    parsers_reg = subparsers.add_parser('reg', help='Show register state')
    # reset subcommand with no arguments
    parsers_reset = subparsers.add_parser('reset', help='Reset the CPU')
    # single step n times
    parsers_singlestep = subparsers.add_parser('singlestep', help='Execute single step instructions')
    # has start address (hex) and length (decimal) arguments
    parsers_singlestep.add_argument('num', type=int, help='Number of single steps to execute (decimal)', default=1)

    args = parser.parse_args()

    last_com_port = None
    if args.port is None:
        coms = comports()
        for c in coms:
            last_com_port = c.device
        if last_com_port is None:
            print("-- Error: No COM ports found, please specify one with -p --")
            exit(-1)
        print("-- Auto-detected COM port: " + str(last_com_port) + " -- ")
    else:
        last_com_port = args.port

    print(f"-- Opening COM port {last_com_port} at {args.baud} bps --")
    ser = serial.Serial(last_com_port, baudrate=args.baud, timeout=0.3) # or whatever port you need
    # do common reset via DTR
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(0.5)

    # wait until firmware has booted up
    content = ""
    print("-- Waiting for firmware bootup (press RESET if no progress) --")
    max_timeout_seconds = 30
    start_time = time.time()
    while True:
        rx = getc(4096, timeout=10)
        if rx != None and len(rx)> 0:
            content += rx.decode('utf-8', errors='ignore')
            print(rx.decode('utf-8', errors="ignore"), end='')
            if "ZDI up" in content:
                break
            if "ZDI down" in content:
                print("-- ZDI connection not established, please check cables and power cycle the device. --")
                exit(-1)
        if time.time() - start_time > max_timeout_seconds:
            print(f"-- Timeout waiting for firmware bootup after {max_timeout_seconds} seconds, exiting. --")
            exit(-1)            

    print("-- Firmware booted up --")

    if args.subparser == 'flash':
        # check if file ends with .bin, we can only flash that.
        if not args.file.endswith('.bin'):
            print("-- Error: Only .bin files are supported --")
            exit(-1)

        # read file argument into a bytes.io Stream
        with open(args.file, 'rb') as f:
            firmware_data = f.read()
        # compute CRC32 of the firmware data
        crc32 = hex(zlib.crc32(firmware_data) & 0xffffffff)
        print(f"-- Firmware file '{args.file}' loaded, size: {len(firmware_data)} bytes, CRC32 {crc32} --")

        if len(firmware_data) > 128*1024:
            print("-- Error: Firmware file too large, max size is 128KB for EZ80F92 --")
            exit(-1)

        # turn up verbosity and make prints show 
        #import logging
        #logging.getLogger('xmodem.XMODEM').setLevel(logging.DEBUG)
        #logging.getLogger('xmodem.XMODEM').addHandler(logging.StreamHandler())

        modem = XMODEM(getc, putc, mode="xmodem")
        print("-- Initiating firmware upload... --")
        putc(f"w{len(firmware_data)}".encode('utf-8'))
        ser.flush()
        print("-- Waiting for firmware to accept XModem upload... --")
        start_time = time.time()
        while True:
            rx = getc(4096, timeout=10)
            if rx != None and len(rx)> 0:
                content += rx.decode('utf-8')
                print(rx.decode('utf-8'), end='')
                if "via XModem now!" in content:
                    break
            if time.time() - start_time > max_timeout_seconds:
                print(f"-- Timeout waiting for firmware to accept XModem upload. --")
                exit(-1)

        print("-- Starting XMODEM transfer... --")
        data_to_send = io.BytesIO(firmware_data)
        #data_to_send = io.BytesIO(b"\xFF" * 73912)  # add 4 byte header to identify start of firmware data
        tx_ok = modem.send(data_to_send, retry=16, callback=lambda total_packets, success_count, error_count: print(f"Sent packet {total_packets}, success: {success_count}, errors: {error_count}", end='\r'))
        if not tx_ok:
            print("\n-- Error during XMODEM transfer --")
            # read remaining serial bytes
            rx = ser.read(4096)
            if rx != None and len(rx)> 0:
                print(rx.decode('utf-8'), end='')
            exit(-1)
        print("\n-- Firmware upload complete, waiting for confirmation --")

        upload_successfull: bool = False
        total_rx = ""
        while True:
            rx = ser.readline(1)
            if rx != None and len(rx)> 0:
                rx_str = rx.decode('utf-8')
                total_rx += rx_str
                print(rx_str, flush=True, end='')
                if "CRC32 OK" in total_rx:
                    upload_successfull = True
                if "CRC32 ERROR" in total_rx:
                    upload_successfull = False
                if "Upload done!" in total_rx:
                    break
        print("\n" if not total_rx.endswith("\n") else "", end='')

        if args.reset:
            # reset is done by sending just the "p" character
            print("-- Sending CPU reset command... --")
            ser.write(b"p\n")
            ser.flush()
            time.sleep(0.5)

        print(f"-- Firmware upload {'successful' if upload_successfull else 'failed'} --")
        print("-- Exiting Flasher --")

        # make exit code negative to indicate failure of the flasher
        if not upload_successfull:
            exit(-1)
    elif args.subparser == 'read':
        # we can execute a read by sending "r {start in decimal} {length in decimal}\n" command
        command = f"r{args.start} {args.length}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        print(f"-- Sent memory read command: {command.strip()} --")
        # read until we get the expected amount of data
        total_data = bytearray()
        start_time = time.time()
        max_timeout_seconds = 10
        while len(total_data) < args.length:
            rx = ser.read(args.length - len(total_data))
            if rx != None and len(rx) > 0:
                total_data.extend(rx)
            if time.time() - start_time > max_timeout_seconds:
                print(f"-- Timeout waiting for memory read data after {max_timeout_seconds} seconds, exiting. --")
                exit(-1)
        print(f"-- Received {len(total_data)} bytes of memory data --")
        # dump data as hex with address info
        for i in range(0, len(total_data), 16):
            chunk = total_data[i:i+16]
            hex_bytes = ' '.join(f"{b:02X}" for b in chunk)
            ascii_bytes = ''.join((chr(b) if 32 <= b < 127 else '.') for b in chunk)
            print(f"{args.start + i:08X}  {hex_bytes:<48}  {ascii_bytes}")
    elif args.subparser == 'reg':
        # send just the letter "d"
        ser.write(b"d\n")
        ser.flush()
        print(f"-- Sent register read command --")
        # read until the line "=== END OF CPU DUMP === "
        total_data = ""
        start_time = time.time()
        max_timeout_seconds = 10
        while True:
            rx = ser.readline()
            if rx != None and len(rx) > 0:
                line = rx.decode('utf-8')
                total_data += line
                print(line, end='')
                if "=== END OF CPU DUMP === " in line:
                    break
            if time.time() - start_time > max_timeout_seconds:
                print(f"-- Timeout waiting for register dump data after {max_timeout_seconds} seconds, exiting. --")
                exit(-1)
    elif args.subparser == 'singlestep':
        # send "s {num}\n"
        command = f"s{args.num}\n"
        ser.write(command.encode('utf-8'))
        ser.flush()
        print(f"-- Sent single step command: {command.strip()} --")
        # read until we get confirmation
        total_data = ""
        start_time = time.time()
        max_timeout_seconds = 10
        while True:
            rx = ser.readline()
            if rx != None and len(rx) > 0:
                line = rx.decode('utf-8')
                total_data += line
                print(line, end='')
                if "Single stepping done." in line:
                    break
            if time.time() - start_time > max_timeout_seconds:
                print(f"-- Timeout waiting for single step confirmation after {max_timeout_seconds} seconds, exiting. --")
                exit(-1)
    elif args.subparser == 'reset':
        print("-- Sending CPU reset command... --")
        ser.write(b"p\n")
        ser.flush()
        time.sleep(0.5)
        print("-- CPU reset command sent --")

if __name__ == "__main__":
    main()