import os

for msg_file in os.listdir():
    if "msg" not in msg_file or "Array" in msg_file:
        continue

    with open(msg_file, "r") as f:
        msg = f.readlines()
    for line in msg:
        if "data" in line and "#" not in line:
            dtype = line.split(" ")[0]
            data = line.split(" ")[1].strip()
            print(msg_file, dtype, data)

    with open(msg_file.replace(".msg", "FixedArray.msg"), "w") as f:
        f.write(f"{dtype}[64] {data}\n")

    with open(msg_file.replace(".msg", "BoundedDynArray.msg"), "w") as f:
        f.write(f"{dtype}[<=64] {data}\n")

    with open(msg_file.replace(".msg", "UnboundedDynArray.msg"), "w") as f:
        f.write(f"{dtype}[] {data}\n")

