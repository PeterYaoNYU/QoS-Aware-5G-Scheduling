import re

log_file_path = './configs/single_1.log'

# Function to read log data from a file
def read_log_file(file_path):
    with open(file_path, 'r') as file:
        return file.read()

# Reading log data from the file
log_data = read_log_file(log_file_path)

# Function to calculate satisfaction rate for each paragraph
def calculate_satisfaction_rate(log_data):
    in_block = False
    begin_pattern = r'remaining win: 1  dataToTransmitInWindow\[0\][\s\S]*?dataToTransmitInWindow\[99\]=\d+'
    # end_pattern = r'remaining win: 99  dataToTransmitInWindow\[0\][\s\S]*?dataToTransmitInWindow\[99\]=\d+'
    blocks = re.findall(begin_pattern, log_data)

    print("number of timesteps: " + str(len(blocks)))
    satisfaction_rates = []
    
    for block in blocks:
        numbers = re.findall(r'dataToTransmitInWindow\[\d+\]=(.*)', block)
        numbers = [int(num) for num in numbers]
        print(numbers)

        satisfied_ues = sum(1 for num in numbers if num == 0)
        print(satisfied_ues)
        satisfaction_rate = satisfied_ues / 100.0  # assuming 100 UEs per block
        satisfaction_rates.append(satisfaction_rate)

    return satisfaction_rates

# Example usage
satisfaction_rates = calculate_satisfaction_rate(log_data)
for i, rate in enumerate(satisfaction_rates, start=1):
    print(f"Satisfaction rate for second {i}: {rate:.2%}")
