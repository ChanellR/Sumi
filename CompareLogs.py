
def Registers (byte_arr: bytes):
    print("r0: " + byte_arr[3:4].hex() + byte_arr[2:3].hex() +"_"+ byte_arr[1:2].hex() + byte_arr[0:1].hex())
    for reg in range(1, 16):
        print(f"r{reg}: " + byte_arr[reg*4+3:reg*4-1:-1].hex("_", 2))
    print(f"cpsr: " + byte_arr[16*4+3:16*4-1:-1].hex("_", 2))
    print(f"spsr: " + byte_arr[17*4+3:17*4-1:-1].hex("_", 2))

def Compare(test_path, key_path):
    base = open(test_path, "rb")
    key = open(key_path, "rb") 
    instructions_counted = 0
    while (b := base.read(4 * 18)):
        k = key.read(4 * 18)
        if b != k and instructions_counted > 930:
           print("base: ")
           Registers(b)
           print("key: ")
           Registers(k)
           print(f"difference after PC: 0x" + b[15*4+3:15*4-1:-1].hex("_", 2))
           print(hex((4 * instructions_counted * 18)) + " bytes in")
           print(f"{instructions_counted} instructions counted")
           exit(0)
        instructions_counted += 1
           
    print("The files are similar")
    
if __name__ == "__main__":
    Compare("logs/my_armwrestler_boot_log.bin", "logs/armwrestler-boot-log.bin")