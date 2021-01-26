'''
removechars.py
HELPER CODE

Removes carriage returns from a file, because switching between Linux and Windows editors all the
time causes Python indentation errors :'^{
'''
FILENAME = 'filename.py'
with open(FILENAME, 'rb+') as f:
    content = f.read()
    f.seek(0)
    f.write(content.replace(b'\r', b''))
    f.truncate()
