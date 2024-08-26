# Read the log file
with open('drone_log.log', 'r') as log_file:
    logs = log_file.readlines()

# Process each line
for line in logs:
    print(line.strip())  # Print each log line