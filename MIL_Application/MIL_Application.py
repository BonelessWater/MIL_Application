import asyncio
import time

class SerialDriver:
    """
    Base class representing a high-level serial driver.
    """

    def __init__(self):
        self.kill_status = False

        # List of length 7 for each thruster
        # Use self.thrust_status_backup for when robot is killed
        self.thrust_status = [0, 0, 0, 0, 0, 0, 0]
        self.thrust_status_backup = [0, 0, 0, 0, 0, 0, 0]

        # Used to keep track of ho much time has passed since last heartbeat
        self.time_since_beat = time.perf_counter()

        # Driver only has to return_kill_status directly after the get_kill_status call
        # This variable will help determine if a get_kill_status call just occured.
        self.return_kill_status = False

        # Driver only has to return_thruster_status directly after the set_thruster call
        # This variable will help determine if a set_thruster call just occured.
        self.return_thruster_status = False

    async def send(self, data: bytes) -> None:

        list_data = list(data)

        # Verifys that packet has the appropiate starting bytes and has atleast four bytes
        #if list_data[0:2] == 'GD' and len(list_data) > 3:
        #    pass
        #else:
        #    return

        # Verifys that packet has correct checksum
        if data == findChecksum_4bytes(data, 8):
            pass
        else:
            return

        # Activates kill_status if more than 1 second has passed since last beat
        # Sets self.tus to True to indicate that the program has to return
        # its kill_status upon next function call.
        if list_data[2] == 0x02:
            self.return_kill_status = True
            if time.perf_counter() - self.time_since_beat > 1:
                self.return_kill_status = True
                self.thrust_status_backup = self.thrust_status
                self.thrust_status = [0,0,0,0,0,0,0]
            else:
                self.return_kill_status = False
        
        # Sending a hearbeat; self.kill_status --> false; 
        # thrusters return to their previous setting; self.time_since_beat is renewed
        if list_data[2] == 0x04:
            data = b"\x47\x44\x04"
            self.kill_status = False
            self.time_since_beat = time.perf_counter()
            self.thrust_status = self.thrust_status_backup

        if list_data[2] == 0x07:
            self.return_thruster_status = True
            thrust_id = int.from_bytes(list_data[2], "big")

            # len(list_data) - 5 because there will be 2 starter, 1 indicator,
            # 1 thruster id and 1 checksum byte
            amount_thrust_commands = len(list_data) - 5

            for i in range(amount_thrust_commands):
                # Thrust value is divided by 255 to obtain a percaentage value
                self.thrust_status[thrust_id] = list_data[i + 5] / 255

    async def receive(self) -> bytes:
        
        # self.return_status to False so this code doesn't cycle unless
        # it is preceeded by another get_kill_status call
        if self.return_kill_status == True and self.kill_status == True:
            return findChecksum_4bytes(b"\x47\x44\x03", 8)
        elif self.return_kill_status == True and self.kill_status == False:
            return findChecksum_4bytes(b"\x47\x44\x03\x01", 8)

        if self.return_thruster_status == True:
            return findChecksum_4bytes(b"\x47\x44\x00", 8)
        else:
            return findChecksum_4bytes(b"\x47\x44\x01", 8)

def findChecksum_4bytes(data, k):

    SentMessage = ''
    for i in range(3):
        SentMessage += format(data[i], f'0{k}b')  

    # Dividing sent message in packets of k bits.
    c1 = SentMessage[0:k]
    c2 = SentMessage[k:2*k]
    c3 = SentMessage[2*k:3*k]
 
    # Calculating the binary sum of packets
    checksum = '00000000'
    bitwise  = '11111111'

    # Checksum algorithm https://en.wikipedia.org/wiki/BSD_checksum
    checksum = circular_shift_right(checksum)
    added_binary = add_binary_numbers(checksum, c1)
    checksum = bitwise_and(added_binary, bitwise)

    checksum = circular_shift_right(checksum)
    added_binary = add_binary_numbers(checksum, c2)
    checksum = bitwise_and(added_binary, bitwise)

    checksum = circular_shift_right(checksum)
    added_binary = add_binary_numbers(checksum, c3)
    checksum = bitwise_and(added_binary, bitwise)

    # Convert final checksum to a byte and append to data packet
    data += bytes([int(checksum, 2)])

    return data

def add_binary_numbers(binary1, binary2):

    # Convert binary strings to integers
    decimal1 = int(binary1, 2)
    decimal2 = int(binary2, 2)

    # Add the decimal numbers
    result_decimal = decimal1 + decimal2

    # Convert the result back to binary
    result_binary = bin(result_decimal)[2:]

    return result_binary
    
def bitwise_and(binary1, binary2, result_length=4):
    # Perform bitwise AND
    result_binary = bin(int(binary1, 2) & int(binary2, 2))[2:]

    # Ensure the result length is not exceeding the limit
    if len(result_binary) > result_length:
        result_binary = result_binary[-result_length:]
    else:
        result_binary = result_binary.zfill(result_length)

    return result_binary

def circular_shift_right(binary_num):
    value = int(binary_num, 2)
    if value % 2 == 1:
        value = (value / 2) - 0.5
        value += 256
    else:
        value /= 2
    print(binary_num)
    return int_to_binary_string(value)

def int_to_binary_string(decimal_num, result_length=8):
    if type(decimal_num) == float:
        decimal_num = int(decimal_num)
    binary_string = bin(decimal_num)[2:]

    # Ensure the result has the specified length
    if result_length:
        binary_string = binary_string.zfill(result_length)

    return binary_string 

async def main():
    driver = SerialDriver()

    # Example 1: Successful get kill status
    # 0x4744 - start of packet
    # 0x02 - packet type
    # 0x35 - checksum
    kill_status_packet = b"\x47\x44\x02\x35"
    await driver.send(kill_status_packet)

    # Return kill status packet
    # 0x4744 - start of packet
    # 0x03 - packet type
    # 0x00 - kill status (not set yet, we have not set it)
    # 0x36 - checksum
    assert await driver.receive() == b"\x47\x44\x03\x36"

    # Wait 1 second (kill will automatically trigger because no heartbeat was
    #               sent)
    await asyncio.sleep(1)

    # Example 2: Successful get kill status
    # 0x4744 - start of packet
    # 0x02 - packet type
    # 0x35 - checksum
    kill_status_packet = b"\x47\x44\x02\x35"
    await driver.send(kill_status_packet)

    # Return kill status packet
    # 0x4744 - start of packet
    # 0x03 - packet type
    # 0x01 - kill status (set because we were not sending heartbeat)
    # 0x1C - checksum
    assert await driver.receive() == b"\x47\x44\x03\x01\x1C"

    # Example 3: Successful thrust set packet
    # 0x4744 - start of packet
    # 0x07 - packet type
    # payload:
    # 0x04 - random thruster value
    # struct.pack("f", 0.31415) --> four bytes, speed packed as a float
    # 0x45 - byte 1 of speed
    # 0xD8 - byte 2 of speed
    # 0xA0 - byte 3 of speed
    # 0x3E - byte 4 of speed
    # 0x8E - checksum
    kill_status_packet = b"\x47\x44\x07\x04\x45\xD8\xA0\x3E\x8E"
    await driver.send(kill_status_packet)

    # Return ACK (kill is still set)
    # 0x4744 - start of packet
    # 0x00 - packet type
    # 0x33 - checksum
    assert await driver.receive() == b"\x47\x44\x00\x33"

    # Example 4: Finally, we can send a heartbeat! :)
    # In reality, you may want to modify this code to send heartbeats automatically,
    # so that your driver does not kill all the time. There are ways to run
    # code periodically in Python using asyncio, many of which are publicly documented
    # and in use at MIL.
    # Anyways...
    # 0x4744 - start of packet
    # 0x04 - packet type
    # 0x1B - checksum
    await driver.send(b"\x47\x44\x04\x1B")

if __name__ == "__main__":
    asyncio.run(main())
