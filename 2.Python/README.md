# Python Data Decoder
Data decoder written in python to retrieve 4-byte-single-precision-based data packet into readable csv (comma-separated values) form.

### Dependency
This is a simple script with no need of external package dependancy :)

### Usage
Just copy the data file from the microSD card in the (successfully) retrieved rocket into the same folder.  
Type the command in your console having the same directory as working dir.  
```console
foo@bar:~$ python sdcard_decoder.py 1234.txt > data.csv
```
Or in windows shell,  
```console
PS foo> python sdcard_decoder.py 1234.txt > data.csv
```
You should get the data if the formats are correctly matched.  
Now let us move to the third stage.

### Credit
[Jaerin Lee](https://github.com/ironjr)  
Lead Electronics and Software Developer  
SNU Rocket Team Hanaro