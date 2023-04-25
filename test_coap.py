import logging
import asyncio

from aiocoap import *

# put your board's IP address here
ESP32_IP = "192.168.39.138"

# un comment the type of test you want to execute
TEST = "GET"
#TEST = "PUT"
#TEST = "DELETE"

#URI = "Espressif/direction"
#URI = "Espressif/speed"
#URI = "Espressif/current"
#URI = "Espressif/temperature"
URI = "Espressif/gas"

PAYLOAD = b"JK"

logging.basicConfig(level=logging.INFO)

async def get(ip, uri):
    protocol = await Context.create_client_context()
    request = Message(code = GET, uri = 'coap://' + ip + '/' +  uri)
    try:
        response = await protocol.request(request).response
    except Exception as e:
        print('Failed to fetch resource:')
        print(e)
    else:
        print('Result: %s\n%r'%(response.code, response.payload))

        if (URI == "Espressif/direction"):
            if (response.payload == b"F"):
                print('Direction: Front \r\n')
            elif (response.payload == b"B"):
                print('Direction: Back \r\n')
            elif (response.payload == b"L"):
                print('Direction: Left \r\n')
            elif (response.payload == b"R"):
                print('Direction: Right \r\n')
            else:
                print('Direction: DIRECTION ERROR \r\n')

        if (URI == "Espressif/speed"):
            print('Velocidad - Duty Cycle: %r %%\r\n'%(response.payload))

        if (URI == "Espressif/temperature"):
            print('Temperatura Ambiente: %r C\r\n'%(response.payload))

        if (URI == "Espressif/current"):
            print('Consumo de Corriente: %r mA\r\n'%(response.payload))


async def put(ip, uri, payload):
    context = await Context.create_client_context()
    await asyncio.sleep(2)
    request = Message(code = PUT, payload = payload, uri = 'coap://' + ip +'/' + uri)
    response = await context.request(request).response
    print('Result: %s\n%r'%(response.code, response.payload))

async def delete(ip, uri):
    context = await Context.create_client_context()
    await asyncio.sleep(2)
    request = Message(code = DELETE, uri = 'coap://' + ip +'/' + uri)
    response = await context.request(request).response
    print('Result: %s\n%r'%(response.code, response.payload))

if __name__ == "__main__":
  asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

  if(TEST == "GET"):
    print("*** GET ***")
    asyncio.run(get(ESP32_IP, URI))
  if(TEST == "PUT"):
    print("*** PUT ***")
    asyncio.run(put(ESP32_IP, URI, PAYLOAD))
    print("*** GET ***")
    asyncio.run(get(ESP32_IP, URI))
  if(TEST == "DELETE"):
    print("*** DELETE ***")
    asyncio.run(delete(ESP32_IP, URI))
    print("*** GET ***")
    asyncio.run(get(ESP32_IP, URI))

