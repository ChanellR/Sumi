if __name__ == "__main__":
    with open("table.txt") as f:
        table_text = f.readline()
        items = table_text.split(", ")
        print(items)
        for item in items:
            print("case "+item+":", end=" ")
        