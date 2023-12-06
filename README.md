# Arduino
You can compile the Arduino library locally, all libraries necessary are included in this reporsitory, all you need is a Nano 33 IoT and copy the libraries into your Arduino libraries folder. 
Modify the ino file and the secrets.h with your specific needs, and it should generate valid w3bstream packets encoded using the protobuf encoding.

Alternatively, you can use http or pub_client.

# http or pub_client 
Make sure you have the following tools installed:

```
sudo apt install httpie
sudo apt install python3-pip
pip install --upgrade pip setuptools
pip install --upgrade httpie
sudo pip install httpie-bearer-auth
```

in a linux terminal, from the root of this project, use the command

`source .env`

this will output some text - these are commands you can use to trigger w3bstream events.




