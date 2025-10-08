#!/bin/bash
# Start Hrafnar in headless mode with all services
# Camera feed accessible via ROS2 + WebRTC visualization

set -e

echo "============================================"
echo "  Hrafnar Headless Mode Startup"
echo "============================================"
echo ""
echo "Starting services:"
echo "  ✓ Isaac Sim (headless with WebRTC)"
echo "  ✓ ROS2 Bridge"
echo "  ✓ Foxglove Bridge (port 8765)"
echo "  ✓ VLM Controller (port 8002)"
echo ""

# Build Isaac Sim image if needed
echo "Building Isaac Sim container (if needed)..."
docker compose build isaac_sim

echo ""
echo "Starting all services..."
docker compose up -d

echo ""
echo "Waiting for services to initialize..."
sleep 10

echo ""
echo "============================================"
echo "  Services Running!"
echo "============================================"
echo ""
echo "📺 Visualization:"
echo "   WebRTC GUI: http://localhost:8211/streaming/webrtc-client/"
echo "   Foxglove:   ws://localhost:8765"
echo ""
echo "🔌 APIs:"
echo "   VLM Control: http://localhost:8002"
echo "   ROSboard:    http://localhost:8888"
echo ""
echo "📸 Camera Feed:"
echo "   Topic: /drone1/camera/color/image_raw"
echo ""
echo "View logs:"
echo "   docker compose logs -f isaac_sim"
echo "   docker compose logs -f vlm_controller"
echo ""
echo "Stop all services:"
echo "   docker compose down"
echo ""
