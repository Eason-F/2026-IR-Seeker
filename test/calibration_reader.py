import csv
from io import StringIO

sensor_number = int(input("Enter sensor number to analyze (1-18): "))

if not 1 <= sensor_number <= 18:
    raise ValueError("Sensor number must be between 1 and 18.")

print("Paste the IR data below.")
print("Press Enter on an empty line when finished.\n")

lines = []

while True:
    line = input()
    if not line.strip():
        break
    lines.append(line)

data = "\n".join(lines)

values = []

for row in csv.reader(StringIO(data)):
    if not row or row[0] != "IR":
        continue

    # Each sensor has:
    # sensor number, raw count, percentage, calibrated strength
    index = 1

    while index + 3 < len(row):
        try:
            sensor = int(row[index])
            percentage = float(row[index + 2])
        except ValueError:
            break

        if sensor == sensor_number:
            values.append(percentage)
            break

        index += 4

if not values:
    print(f"\nNo readings found for sensor {sensor_number}.")
else:
    average = sum(values) / len(values)

    print(f"\nSensor {sensor_number}")
    print(f"Readings: {len(values)}")
    print(f"Average detection: {average:.2f}%")