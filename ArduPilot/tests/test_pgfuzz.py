import unittest
import subprocess
import time
import os


class TestPGFuzz(unittest.TestCase):
    def setUp(self):
        self.pgfuzz_path = os.path.join(os.path.dirname(__file__), "..", "pgfuzz.py")
        self.timeout = 15 * 60  # 15 minutes in seconds
        self.config1 = os.path.join(os.path.dirname(__file__), "gimbal.ini")
        self.config2 = os.path.join(os.path.dirname(__file__), "range.ini")

    def run_pgfuzz_test(self, config_file):
        print(f"\nRunning test with config: {config_file}")
        start_time = time.time()

        process = subprocess.Popen(["python3", self.pgfuzz_path, config_file])

        while True:
            # Check if process has finished
            retcode = process.poll()
            if retcode is not None:
                # Process finished - check exit code
                self.assertEqual(retcode, 0, f"Expected exit code 0, got {retcode}")
                break

            # Check timeout
            if time.time() - start_time >= self.timeout:
                process.kill()
                self.fail(f"Test timed out after {self.timeout} seconds")
                break

            time.sleep(1)

    def test_pgfuzz_config1(self):
        self.run_pgfuzz_test(self.config1)

    def test_pgfuzz_config2(self):
        self.run_pgfuzz_test(self.config2)


if __name__ == "__main__":
    unittest.main()
