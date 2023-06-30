import csv

class functions:

    def __init__(self,name):
        self.name=name
    
    def write_data(self,points,timestamp):

        with open(self.name, 'a', newline=' ') as csvfile:
            
            spamwriter = csv.writer(csvfile, delimiter=' ')
            
            spamwriter.writerow(['Spam'] * 5 + ['Baked Beans'])
            spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])