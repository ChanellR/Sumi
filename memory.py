if __name__ == "__main__":
    with open("table.txt") as f:
        table_text = f.readlines()
        # print(table_text[1].strip())
        for line in table_text:
            new = line.strip()
            if new[0] == "0":
                bounds = f"else if(addr >= 0x{new[:8]} && addr <= 0x{new[9:17]})" + "{}"
                print(bounds)