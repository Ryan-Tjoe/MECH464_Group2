import asyncio
import websockets
import json

class WebSocketClient:
    def __init__(self, uri):
        self.uri = uri  # The WebSocket server URI

    async def send_message(self, message):
        """
        Connects to the WebSocket server and sends a message.
        """
        async with websockets.connect(self.uri) as websocket:
            # Serialize the message as a JSON string
            message_json = json.dumps(message)
            print(f"Sending: {message_json}")
            await websocket.send(message_json)

            # Receive the response from the server
            response = await websocket.recv()
            print(f"Received: {response}")
            return response

    async def send_json(self, data):
        """
        Creates a JSON object and sends it to the WebSocket server.
        """
        message = {
            "data": data
        }
        return await self.send_message(message)

# Example usage of the WebSocketClient class
async def main():
    uri = "ws://localhost:8765"
    client = WebSocketClient(uri)

    # Sending a custom JSON object
    message = {
        "name": "Alice",
        "age": 30,
        "city": "New York"
    }

    response = await client.send_message(message)
    print(f"Server Response: {response}")

    # Sending another JSON object via send_json method
    response = await client.send_json({"action": "test"})
    print(f"Server Response: {response}")

# Run the client
asyncio.run(main())
