#!/usr/bin/env python3
import asyncio
from aiohttp import web

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


BOUNDARY = "frame"


class MjpegServer(Node):
    def __init__(self):
        super().__init__("mjpeg_server")

        self.declare_parameter("topic", "ui/zed/rgb")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("jpeg_content_type", "image/jpeg")

        self.topic = str(self.get_parameter("topic").value)
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.jpeg_ct = str(self.get_parameter("jpeg_content_type").value)

        self._latest = None

        self.create_subscription(CompressedImage, self.topic, self._on_img, 10)

        self._app = web.Application()
        self._app.router.add_get("/", self._handle_root)
        self._app.router.add_get("/stream", self._handle_stream)

        self.get_logger().info(
            f"MJPEG server: http://{self.host}:{self.port}/stream topic={self.topic}"
        )

    def _on_img(self, msg: CompressedImage):
        self._latest = bytes(msg.data)

    async def _handle_root(self, request):
        return web.Response(text="OK. Use /stream")

    async def _handle_stream(self, request):
        resp = web.StreamResponse(
            status=200,
            reason="OK",
            headers={
                "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0",
            },
        )
        await resp.prepare(request)

        min_dt = 1.0 / max(1e-6, self.max_fps)

        try:
            while True:
                if self._latest is None:
                    await asyncio.sleep(0.05)
                    continue

                frame = self._latest
                chunk = (
                    f"--{BOUNDARY}\r\n"
                    f"Content-Type: {self.jpeg_ct}\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                ).encode("utf-8") + frame + b"\r\n"

                await resp.write(chunk)
                await asyncio.sleep(min_dt)

        except (asyncio.CancelledError, ConnectionResetError, BrokenPipeError):
            pass
        finally:
            return resp

    async def start(self):
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()


def main():
    rclpy.init()
    node = MjpegServer()

    loop = asyncio.get_event_loop()
    loop.create_task(node.start())

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            loop.run_until_complete(asyncio.sleep(0.0))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()