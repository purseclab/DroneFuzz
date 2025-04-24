#!/usr/bin/env python3
"""
Merge Request Analyzer

This script analyzes JSON files containing merge request information by:
1. Processing JSON files with merge request data
2. Calling Claude 3.5 to analyze each merge request
3. Sorting/filtering entries based on bug severity and reproducibility
"""

import argparse
import json
import os
import sys
from pathlib import Path
import anthropic
import logging
from typing import List, Dict, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)

class MergeRequestAnalyzer:
    """Analyzes merge requests using Claude 3.5 to identify critical bugs."""
    
    def __init__(self, api_key: str):
        """
        Initialize the analyzer with API credentials.
        
        Args:
            api_key: Anthropic API key for Claude
        """
        self.client = anthropic.Anthropic(api_key=api_key)
        self.system_prompt = """
You are an expert in drone software analysis. Your task is to analyze merge requests for drone software 
and determine if they meet BOTH of these conditions:

1. The merge request fixes a bug that could have physical consequences or cause the drone to crash
2. The bug is reproducible in SITL (Software In The Loop simulation) with minor modifications (10 lines of code or less)

For each merge request, analyze:
- The title and description
- The modified files and their changes
- The potential impact on drone safety
- The reproducibility in simulation

Respond with a JSON object containing:
{
  "meets_criteria": true/false,
  "reasoning": "Your detailed analysis explaining why it meets or fails the criteria",
  "crash_potential": "High/Medium/Low - explanation of the physical consequences",
  "reproducibility": "Easy/Moderate/Difficult - explanation of how to reproduce in SITL",
  "estimated_modification_lines": number
}
"""

    def analyze_merge_request(self, merge_request: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyze a single merge request using Claude 3.5.
        
        Args:
            merge_request: Dictionary containing merge request data
            
        Returns:
            Dictionary with the original merge request and Claude's analysis
        """
        # Prepare the prompt with merge request details
        prompt = f"""
Merge Request Analysis:

Title: {merge_request.get('title', 'No title')}
URL: {merge_request.get('url', 'No URL')}

Description:
{merge_request.get('body', 'No description')}

Modified Files:
{json.dumps(merge_request.get('files', []), indent=2)}

Commit: {merge_request.get('mergeCommit', 'No commit info')}
"""

        try:
            # Call Claude API
            response = self.client.messages.create(
                model="claude-3-5-sonnet-20240620",
                max_tokens=4000,
                system=self.system_prompt,
                messages=[
                    {"role": "user", "content": prompt}
                ]
            )
            
            # Extract and parse the JSON response
            content = response.content[0].text
            
            # Try to extract JSON from the response
            try:
                # Look for JSON block in the response
                import re
                json_match = re.search(r'```json\n(.*?)\n```', content, re.DOTALL)
                if json_match:
                    analysis = json.loads(json_match.group(1))
                else:
                    # Try to parse the whole response as JSON
                    analysis = json.loads(content)
            except json.JSONDecodeError:
                logger.warning(f"Failed to parse JSON from Claude's response. Using raw response.")
                analysis = {
                    "meets_criteria": False,
                    "reasoning": "Failed to parse Claude's response",
                    "raw_response": content
                }
            
            # Combine original merge request with analysis
            result = merge_request.copy()
            result["analysis"] = analysis
            return result
            
        except Exception as e:
            logger.error(f"Error calling Claude API: {str(e)}")
            return {
                **merge_request,
                "analysis": {
                    "meets_criteria": False,
                    "reasoning": f"API error: {str(e)}",
                    "error": True
                }
            }

    def process_json_file(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Process a JSON file containing merge requests.
        
        Args:
            file_path: Path to the JSON file
            
        Returns:
            List of processed merge requests with analysis
        """
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            if not isinstance(data, list):
                logger.error(f"Expected a list in {file_path}, but got {type(data)}")
                return []
            
            results = []
            total = len(data)
            
            for i, merge_request in enumerate(data):
                logger.info(f"Processing merge request {i+1}/{total}: {merge_request.get('title', 'Untitled')}")
                analyzed = self.analyze_merge_request(merge_request)
                results.append(analyzed)
            
            return results
            
        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            return []

    def sort_results(self, results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Sort the results based on criteria match and crash potential.
        
        Args:
            results: List of analyzed merge requests
            
        Returns:
            Sorted list of merge requests
        """
        # Define a sorting key function
        def sort_key(item):
            analysis = item.get("analysis", {})
            meets_criteria = analysis.get("meets_criteria", False)
            
            # Extract crash potential and convert to numeric value
            crash_potential = analysis.get("crash_potential", "Low")
            if isinstance(crash_potential, str):
                if crash_potential.startswith("High"):
                    crash_value = 3
                elif crash_potential.startswith("Medium"):
                    crash_value = 2
                else:
                    crash_value = 1
            else:
                crash_value = 0
                
            # Extract reproducibility and convert to numeric value
            reproducibility = analysis.get("reproducibility", "Difficult")
            if isinstance(reproducibility, str):
                if reproducibility.startswith("Easy"):
                    repro_value = 3
                elif reproducibility.startswith("Moderate"):
                    repro_value = 2
                else:
                    repro_value = 1
            else:
                repro_value = 0
                
            # Primary sort by meets_criteria, then by crash potential, then by reproducibility
            return (not meets_criteria, -crash_value, -repro_value)
        
        return sorted(results, key=sort_key)

    def save_results(self, results: List[Dict[str, Any]], output_path: str) -> None:
        """
        Save the analyzed and sorted results to a JSON file.
        
        Args:
            results: List of analyzed merge requests
            output_path: Path to save the results
        """
        try:
            with open(output_path, 'w') as f:
                json.dump(results, f, indent=2)
            logger.info(f"Results saved to {output_path}")
        except Exception as e:
            logger.error(f"Error saving results to {output_path}: {str(e)}")


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description="Analyze merge requests using Claude 3.5")
    parser.add_argument("input_files", nargs="+", help="JSON files containing merge requests")
    parser.add_argument("--output", "-o", default="analyzed_merge_requests.json", 
                        help="Output file for analyzed results")
    parser.add_argument("--api-key", help="Anthropic API key (or set ANTHROPIC_API_KEY env var)")
    
    args = parser.parse_args()
    
    # Get API key from args or environment
    api_key = args.api_key or os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        logger.error("Anthropic API key not provided. Use --api-key or set ANTHROPIC_API_KEY environment variable.")
        sys.exit(1)
    
    analyzer = MergeRequestAnalyzer(api_key)
    all_results = []
    
    for input_file in args.input_files:
        logger.info(f"Processing file: {input_file}")
        results = analyzer.process_json_file(input_file)
        all_results.extend(results)
    
    sorted_results = analyzer.sort_results(all_results)
    analyzer.save_results(sorted_results, args.output)
    
    # Print summary
    critical_bugs = sum(1 for r in sorted_results if r.get("analysis", {}).get("meets_criteria", False))
    logger.info(f"Analysis complete. Found {critical_bugs} critical bugs out of {len(sorted_results)} merge requests.")
    logger.info(f"Results saved to {args.output}")


if __name__ == "__main__":
    main()
