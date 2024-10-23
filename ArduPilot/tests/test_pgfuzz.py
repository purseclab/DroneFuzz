import unittest
import subprocess
import time
import os
import shutil


class TestPGFuzz(unittest.TestCase):
    def setUp(self):
        self.pgfuzz_path = os.path.join(os.path.dirname(__file__), "..", "pgfuzz.py")
        self.policy_violations_dir = os.path.join(
            os.path.dirname(__file__), "..", "policy_violations"
        )
        self.timeout = 30 * 60  # 30 minutes in seconds
        self.config1 = os.path.join(os.path.dirname(__file__), "gimbal.ini")
        print(self.config1)
        self.config2 = os.path.join(os.path.dirname(__file__), "range.ini")
        print(self.config2)

    def clear_policy_violations(self):
        if os.path.exists(self.policy_violations_dir):
            shutil.rmtree(self.policy_violations_dir)
        os.makedirs(self.policy_violations_dir)

    def count_reports(self):
        return len(
            [f for f in os.listdir(self.policy_violations_dir) if f.endswith(".txt")]
        )

    def run_pgfuzz_test(self, config_file):
        self.clear_policy_violations()
        start_time = time.time()

        # Start pgfuzz.py as a subprocess with the specified config
        process = subprocess.Popen(["python3", self.pgfuzz_path, config_file])

        try:
            while True:
                if time.time() - start_time >= self.timeout:
                    self.assertLess(
                        self.count_reports(),
                        2,
                        f"PGFuzz with {config_file} ran for 30 minutes but didn't generate 2 reports",
                    )
                    break

                if self.count_reports() >= 2:
                    elapsed_time = time.time() - start_time
                    self.assertLess(
                        elapsed_time,
                        self.timeout,
                        f"PGFuzz with {config_file} generated 2 reports before the 30-minute timeout",
                    )
                    break
                time.sleep(1)  # Check every second
        finally:
            # Ensure the process is terminated
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)  # Give it 10 seconds to terminate gracefully
                if process.poll() is None:
                    process.kill()  # Force kill if it's still running

    def test_pgfuzz_config1(self):
        self.run_pgfuzz_test(self.config1)

    def test_pgfuzz_config2(self):
        self.run_pgfuzz_test(self.config2)


if __name__ == "__main__":
    unittest.main()
