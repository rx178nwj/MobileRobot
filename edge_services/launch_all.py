#!/usr/bin/env python3
"""
Launch All Edge Services

Starts all edge services (motor, odometry, camera) in a single process.
"""

import asyncio
import logging
import json
import signal
import sys
from pathlib import Path

# Import services
from motor_service import MotorControlService
from odometry_service import OdometryService
from camera_service import CameraService
from hardware.pr2040_usb_driver import PR2040USBDriver


class EdgeServiceLauncher:
    """Launch and manage all edge services"""

    def __init__(self, config_file: str = 'config/robot_config.json'):
        """
        Initialize launcher

        Args:
            config_file: Path to configuration file
        """
        self.logger = logging.getLogger(__name__)

        # Load configuration
        self.config = self._load_config(config_file)

        # Services
        self.motor_service = None
        self.odometry_service = None
        self.camera_service = None

        # Shared hardware driver
        self.shared_driver = None

        # Tasks
        self.tasks = []

    def _load_config(self, config_file: str) -> dict:
        """Load configuration from JSON file"""
        config_path = Path(__file__).parent / config_file

        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            self.logger.info(f"Configuration loaded from {config_path}")
            return config
        except FileNotFoundError:
            self.logger.error(f"Configuration file not found: {config_path}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON in configuration file: {e}")
            sys.exit(1)

    def _setup_logging(self):
        """Setup logging configuration"""
        log_config = self.config.get('logging', {})
        level = getattr(logging, log_config.get('level', 'INFO'))

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        console_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(console_formatter)

        # File handler (if configured)
        handlers = [console_handler]
        log_file = log_config.get('file')

        if log_file:
            log_path = Path(__file__).parent / log_file
            log_path.parent.mkdir(parents=True, exist_ok=True)

            from logging.handlers import RotatingFileHandler
            file_handler = RotatingFileHandler(
                log_path,
                maxBytes=log_config.get('max_bytes', 10485760),
                backupCount=log_config.get('backup_count', 5)
            )
            file_handler.setLevel(level)
            file_handler.setFormatter(console_formatter)
            handlers.append(file_handler)

        # Configure root logger
        logging.basicConfig(
            level=level,
            handlers=handlers
        )

    def _create_motor_config(self) -> dict:
        """Create motor service configuration"""
        robot = self.config['robot']
        motor = self.config['motor_service']
        hardware = self.config['hardware']

        return {
            'wheel_base': robot['wheel_base'],
            'wheel_radius': robot['wheel_radius'],
            'encoder_cpr': robot['encoder_cpr'],
            'max_linear_velocity': robot['max_linear_velocity'],
            'max_angular_velocity': robot['max_angular_velocity'],
            'cmd_vel_timeout': motor['cmd_vel_timeout'],
            'host': motor['host'],
            'port': motor['port'],
            'usb_port': motor.get('usb_port', hardware.get('usb_port', '/dev/ttyACM0'))
        }

    def _create_odometry_config(self) -> dict:
        """Create odometry service configuration"""
        robot = self.config['robot']
        odom = self.config['odometry_service']
        hardware = self.config['hardware']

        return {
            'wheel_base': robot['wheel_base'],
            'wheel_radius': robot['wheel_radius'],
            'encoder_cpr': robot['encoder_cpr'],
            'publish_rate': odom['publish_rate'],
            'host': odom['host'],
            'port': odom['port'],
            'usb_port': odom.get('usb_port', hardware.get('usb_port', '/dev/ttyACM0'))
        }

    def _create_camera_config(self) -> dict:
        """Create camera service configuration"""
        camera = self.config['camera_service']
        return dict(camera)

    async def run(self):
        """Run all enabled services"""
        self.logger.info("=" * 60)
        self.logger.info("Starting Edge Services")
        self.logger.info("=" * 60)

        # Create shared USB driver (motor & odometry share one serial connection)
        usb_port = self.config['hardware'].get('usb_port', '/dev/ttyACM0')
        self.shared_driver = PR2040USBDriver(port=usb_port)
        if self.shared_driver.connected:
            self.logger.info(f"✓ PR2040 USB driver connected at {usb_port}")
        else:
            self.logger.error(f"✗ PR2040 USB driver failed to connect at {usb_port}")

        # Create services
        if self.config['motor_service']['enabled']:
            motor_config = self._create_motor_config()
            motor_config['driver'] = self.shared_driver
            self.motor_service = MotorControlService(motor_config)
            self.tasks.append(asyncio.create_task(self.motor_service.run()))
            self.logger.info("✓ Motor Control Service enabled")

        if self.config['odometry_service']['enabled']:
            odom_config = self._create_odometry_config()
            odom_config['driver'] = self.shared_driver
            self.odometry_service = OdometryService(odom_config)
            self.tasks.append(asyncio.create_task(self.odometry_service.run()))
            self.logger.info("✓ Odometry Service enabled")

        if self.config['camera_service']['enabled']:
            self.camera_service = CameraService(self._create_camera_config())
            asyncio.create_task(self.camera_service.run())
            self.logger.info("✓ Camera Service enabled")

        self.logger.info("=" * 60)
        self.logger.info("All services started successfully")
        self.logger.info("Press Ctrl+C to stop")
        self.logger.info("=" * 60)

        if not self.tasks:
            self.logger.error("No services started successfully")
            return

        # Wait for all tasks (run forever until cancelled)
        await asyncio.gather(*self.tasks, return_exceptions=True)

    def shutdown(self):
        """Shutdown all services"""
        self.logger.info("Shutting down all services...")

        if self.motor_service:
            self.motor_service.shutdown()

        if self.odometry_service:
            self.odometry_service.shutdown()

        if self.camera_service:
            self.camera_service.shutdown()

        if self.shared_driver:
            self.shared_driver.close()

        self.logger.info("All services stopped")


async def main():
    """Main entry point"""
    # Create launcher
    launcher = EdgeServiceLauncher()
    launcher._setup_logging()

    # Setup signal handlers
    def signal_handler(sig, frame):
        logging.info("Signal received, shutting down...")
        launcher.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Run services
    try:
        await launcher.run()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
    finally:
        launcher.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
