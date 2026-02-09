import serial, time, queue, threading

ser = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0)

# Critical Mac/Zephyr handshake
ser.dtr = True
ser.rts = True

time.sleep(1.0)

dataout_queue = {i: queue.Queue() for i in range(16)}

input_queues = {
    "SIPO_DONE": queue.Queue(),
    "DAC_DONE":  queue.Queue(),
    "SRAMOUT": queue.Queue(), 
    "DLAL": queue.Queue()
} 

i = 0 

def serial_listener():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('ascii').strip()
            if not line: continue 

            if line.startswith("DATAOUT:"): 
                try: 
                    _, payload = line.split(":") 
                    board_id, val, ts = map(int, payload.split(",")) 
                    # use put because it is thread safe
                    dataout_queue[board_id].put((val, ts))
                    # test
                    #test_val, test_ts = dataout_queue[0].get() 
                    #print(test_val, test_ts)
                except: 
                    continue
            elif "SIPO_DONE" in line:
                input_queues["SIPO_DONE"].put(True)
            elif "DAC_DONE" in line:
                input_queues["DAC_DONE"].put(True)
            elif line.startswith("SRAMOUT:"):
                input_queues["SRAMOUT"].put(line)
            elif line.startswith("DLAL_RESP:"): 
                input_queues["DLAL"].put(line) 

# Start the listener
threading.Thread(target=serial_listener, daemon=True).start()


def SIPO_data_in(sipo_data0, sipo_data1, sipo_data2, sipo_data3, sipo_data4, sipo_data5, sipo_data6, sipo_data7): 

    cmd = f"SIPO:{sipo_data7:04X},{sipo_data6:04X},{sipo_data5:04X},{sipo_data4:04X},{sipo_data3:04X},{sipo_data2:04X},{sipo_data1:04X},{sipo_data0:04X}\n"
    ser.write(cmd.encode('ascii'))

    try:
        input_queues["SIPO_DONE"].get(timeout=5)
        print(f"SIPO done{i} received!")
    except queue.Empty:
        print("SIPO timeout!") 
    
    return

def DAC_data_in(dac_data2, dac_data1, dac_data0):    
    cmd = f"DAC:{dac_data2:04X},{dac_data1:04X},{dac_data0:04X}\n"

    ser.write(cmd.encode('ascii'))

    try: 
        input_queues["DAC_DONE"].get(timeout=5)
        print(f"DAC done{i} received!") 
    except queue.Empty: 
        print("DAC timeout!") 
    
    return 

def SRAM_data_out():
    SRAMout = 0
    try:
        # Get the actual string from the SRAM bucket
        raw_msg = input_queues["SRAMOUT"].get(timeout=2)
        #print("SRAMOUT found")
        raw_data = raw_msg.replace("SRAMOUT:", "").split(",")
        print(f"raw_data: {raw_data}")
        vals = [int(v, 16) for v in raw_data]
        SRAMout = (vals[6] << 96) + (vals[5] << 80) + (vals[4] << 64) + \
                      (vals[3] << 48) + (vals[2] << 32) + (vals[1] << 16) + vals[0]
        
        # test 
        binary_string = f"{SRAMout:0112b}"
        formatted_bin = "_".join(binary_string[i:i+8] for i in range(0, len(binary_string), 8))
        print(f"SRAM [HEX]: 0x{SRAMout:028X}\n")
        print(f"SRAM [BIN]: {formatted_bin}\n")

        return SRAMout
    except queue.Empty:
        print("SRAM Timeout!")
        return 0

def DL_ALValues(): 
    cmd = "DLAL_RQST\n"
    ser.write(cmd.encode('ascii'))

    try: 
        raw_msg = input_queues["DLAL"].get(timeout=5) 
        raw_data = raw_msg.replace("DLAL_RESP:", "").strip().split(",")

        finalCount = int(raw_data[0])
        finalAddress = int(raw_data[1])
        # test 
        print(f"DL:{finalCount}, AL:{finalAddress}")
        return (finalCount, finalAddress) 
    except queue.Empty: 
        print("DLAL timeout") 
        return None, None
    except (ValueError, IndexError) as e: 
        print(f"DLAL Data Error: Received malformed message. ({e})") 
        return None, None 
        

def Main_data_out(i, boardAddress): 
    if i == 0:
        cmd = "DATAOUT_RST\n" # data_rd_reset
        ser.write(cmd.encode('ascii')) 

    dataout = 0
    timestamp = 0
    
    # check if boardAddress has at least one dataout 
    if not dataout_queue[boardAddress].empty(): 
        dataout, timestamp = dataout_queue[boardAddress].get()
 
    return dataout, timestamp


# if __name__ == "__main__":
#     print("--- Starting DATAOUT Monitoring ---")
#     print("Watching for packets from the Motherboard...")
    
#     i = 0
#     board_address = 0
#     try:
#         while True:
#             # Just keep the script running
#             val, ts = Main_data_out(i, board_address)
#             if val != 0 or ts != 0:
#                 print(f"Board {board_address} | Data: {val} | TS: {ts}")
#             if board_address == 0:
#                 board_address = 1
#             else: 
#                 board_address = 0
#             i+=1
#             time.sleep(0.5)
#     except KeyboardInterrupt:
#         print("\nStopping Test...")

if __name__ == "__main__":
    time.sleep(1) 
    
    print("--- START ---")

    try: 
        while True: 
             time.sleep(2)
             DL_ALValues()
             #SRAM_data_out()
            
    except KeyboardInterrupt:
        print("\nStopping Test...")

