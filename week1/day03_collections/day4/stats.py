import csv
from statistics import mean

def load_column(filename, column):
    values = []
    with open(filename) as f:
        reader = csv.DictReader(f)
        for row in reader:
            values.append(float(row[column]))
    return values

def print_stats(name, values):
    print(f"{name}: count={len(values)}, min={min(values)}, "
          f"max={max(values)}, mean={mean(values):.2f}")

if __name__ == "__main__":
    for col in ["angle_deg", "torque_nm"]:
        print_stats(col, load_column("data.csv", col))