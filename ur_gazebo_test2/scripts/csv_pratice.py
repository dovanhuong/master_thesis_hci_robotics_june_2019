import csv

def write_csv(data):
    with open('example.csv', 'a') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(data)

for x in range(5):
    write_csv([x, x+1])