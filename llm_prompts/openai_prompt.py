import os
from openai import OpenAI
from dotenv import load_dotenv
import tiktoken
import argparse

# Load environment variables
load_dotenv()

# Configure OpenAI API
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY"), organization=os.getenv("OPENAI_ORG_ID")
)

# Price per 1K tokens (as of 2024)
PRICE_INPUT_PER_1K = 0.0005  # GPT-3.5-turbo-0125 input price
PRICE_OUTPUT_PER_1K = 0.0015  # GPT-3.5-turbo-0125 output price


def count_tokens(text, model="gpt-3.5-turbo"):
    """Count the number of tokens in a text string."""
    encoding = tiktoken.encoding_for_model(model)
    return len(encoding.encode(text))


def calculate_cost(input_tokens, output_tokens):
    """Calculate the cost in USD for the API call."""
    input_cost = (input_tokens / 1000) * PRICE_INPUT_PER_1K
    output_cost = (output_tokens / 1000) * PRICE_OUTPUT_PER_1K
    return input_cost + output_cost


def generate_o1_response():
    prompt = """
        You are an expert about drone software. You are supposed to generate policies to detect potential physical issues with the drone.
        Generate Linear Temporal Logic equation that should be maintained while drone is in operation for following sensor: OpticalFlow
    """
    input_tokens = count_tokens(prompt)
    try:
        response = client.chat.completions.create(
            model="o1-preview", messages=[{"role": "user", "content": prompt}]
        )
        # Get response content
        response_content = response.choices[0].message.content

        # Count output tokens
        output_tokens = count_tokens(response_content)

        # Calculate total cost
        total_cost = calculate_cost(input_tokens, output_tokens)

        return {
            "response": response_content,
            "input_tokens": input_tokens,
            "output_tokens": output_tokens,
            "total_cost": total_cost,
        }

    except Exception as e:
        return f"Error: {str(e)}"


def generate_response(system_prompt, user_input):
    """Generate a response using the OpenAI API."""
    model = "gpt-4o"
    temperature = 0.1
    try:
        # Count input tokens
        input_tokens = count_tokens(system_prompt)

        # Make API call
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_input},
            ],
            temperature=temperature,
        )

        print("Used model {0} with temperature {1}".format(response.model, temperature))
        # Get response content
        response_content = response.choices[0].message.content

        # Count output tokens
        output_tokens = count_tokens(response_content)

        # Calculate total cost
        total_cost = calculate_cost(input_tokens, output_tokens)

        return {
            "response": response_content,
            "input_tokens": input_tokens,
            "output_tokens": output_tokens,
            "total_cost": total_cost,
        }

    except Exception as e:
        return f"Error: {str(e)}"


def main():
    parser = argparse.ArgumentParser(description="Generate responses using OpenAI API")
    parser.add_argument("prompt", nargs="?", help="System prompt to send to OpenAI")
    parser.add_argument("input", nargs="?", help="User input to send to OpenAI")
    args = parser.parse_args()

    # Generate response
    # result = generate_response(args.prompt, args.input)
    result = generate_o1_response()

    if isinstance(result, dict):
        print("\nResponse:")
        print("-" * 50)
        print(result["response"])
        print("-" * 50)
        print(f"\nToken Usage:")
        print(f"Input tokens: {result['input_tokens']}")
        print(f"Output tokens: {result['output_tokens']}")
        print(f"Total tokens: {result['input_tokens'] + result['output_tokens']}")
        print(f"Estimated cost: ${result['total_cost']:.4f}")
    else:
        print(result)


if __name__ == "__main__":
    main()
