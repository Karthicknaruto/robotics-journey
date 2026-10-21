import csv
from conversions import deg2rad  # Importing from your own custom module

def process_joint_data(filepath: str):
    # Dictionary to hold lists of valid angles for each joint
    joint_angles = {
        "joint_1": [],
        "joint_2": [],
        "joint_3": []
    }

    try:
        with open(filepath, mode="r", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            
            for row_idx, row in enumerate(reader, start=1):
                for joint_name in joint_angles.keys():
                    raw_val = row.get(joint_name)
                    try:
                        val_deg = float(raw_val)
                        joint_angles[joint_name].append(val_deg)
                    except (ValueError, TypeError):
                        # Catch corrupted data entries cleanly
                        print(f"Warning: Corrupt value '{raw_val}' in {joint_name} at row {row_idx}. Skipped.")

    except FileNotFoundError:
        print(f"Error: The file {filepath} was not found.")
        return

    # Compute and display minimum and maximum values per joint
    print("\n--- Joint Angle Analysis (in Degrees & Radians) ---")
    for joint_name, values in joint_angles.items():
        if values:
            min_deg = min(values)
            max_deg = max(values)
            
            # Convert min and max to radians using imported module
            min_rad = deg2rad(min_deg)
            max_rad = deg2rad(max_deg)
            
            print(f"{joint_name}:")
            print(f"  Min: {min_deg:6.2f}°  ({min_rad:6.3f} rad)")
            print(f"  Max: {max_deg:6.2f}°  ({max_rad:6.3f} rad)")
        else:
            print(f"{joint_name}: No valid data points found.")

if __name__ == "__main__":
    process_joint_data("joint_trajectory.csv")