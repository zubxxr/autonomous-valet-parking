#!/usr/bin/env python3

import yaml

def split_yaml_docs(full_input):
    return [doc.strip() for doc in full_input.split('---') if doc.strip()]

def parse_pose(doc):
    try:
        data = yaml.safe_load(doc)
        x = round(data['pose']['position']['x'], 2)
        y = round(data['pose']['position']['y'], 2)
        z = round(data['pose']['position']['z'], 2)
        oz = round(data['pose']['orientation']['z'], 4)
        ow = round(data['pose']['orientation']['w'], 4)
        return {"x": x, "y": y, "z": z, "oz": oz, "ow": ow}
    except Exception as e:
        print(f"\Failed to parse doc:\n{doc}\nError: {e}")
        return None

def main():
    try:
        expected = int(input("How many parking spots do you have? "))
    except ValueError:
        print("Invalid number.")
        return

    print("\nPaste the full ros2 topic echo message dump below.\nWhen you're done, press Enter *twice*:\n")
    lines = []
    while True:
        try:
            line = input()
            if line.strip() == "":
                break
            lines.append(line)
        except EOFError:
            break

    full_input = "\n".join(lines)
    docs = split_yaml_docs(full_input)

    if len(docs) != expected:
        print(f"\nWarning: You entered {expected} as the expected count, but {len(docs)} entries were detected.\n")

    result = {}
    for i, doc in enumerate(docs, start=1):
        parsed = parse_pose(doc)
        if parsed:
            result[i] = parsed

    print("\nFinal dictionary:\n")
    print("parking_spot_goals = {")
    for k, v in result.items():
        print(f"    {k}: {v},")
    print("}")

if __name__ == "__main__":
    main()

