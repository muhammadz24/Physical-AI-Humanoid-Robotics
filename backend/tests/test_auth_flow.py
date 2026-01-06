#!/usr/bin/env python3
"""
Test script to verify the authentication flow works correctly.
This script will test user signup, signin, and profile access.
"""
import asyncio
import httpx
import json
from typing import Optional

# Configuration
BASE_URL = "http://localhost:8000"  # Adjust this to your backend URL
HEADERS = {"Content-Type": "application/json"}

# Test user data
TEST_USER = {
    "name": "Test User",
    "email": "test_auth_" + str(__import__('time').time()).replace(".", "")[-6:] + "@example.com",  # Unique email
    "password": "testpassword123",
    "software_experience": "beginner",
    "hardware_experience": "none"
}

async def test_signup():
    """Test user signup endpoint."""
    print("Testing signup...")
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/auth/signup",
            headers=HEADERS,
            json=TEST_USER
        )
        print(f"Signup response: {response.status_code}")
        if response.status_code == 201:
            print("+ Signup successful")
            return True
        elif response.status_code == 409:
            print("! User already exists (this is OK for testing)")
            return True  # Already exists is fine for testing
        else:
            print(f"- Signup failed: {response.text}")
            return False

async def test_signin():
    """Test user signin endpoint."""
    print("\nTesting signin...")
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/auth/signin",
            headers=HEADERS,
            json={
                "email": TEST_USER["email"],
                "password": TEST_USER["password"]
            }
        )
        print(f"Signin response: {response.status_code}")
        print(f"Response headers: {dict(response.headers)}")

        if response.status_code == 200:
            print("+ Signin successful")
            # Check if cookies are set
            cookies = response.cookies
            print(f"Cookies set: {list(cookies.jar._cookies.values()) if hasattr(cookies.jar, '_cookies') else 'No cookies accessible'}")
            return True, response.cookies
        else:
            print(f"- Signin failed: {response.text}")
            return False, None

async def test_get_profile(cookies):
    """Test getting user profile with valid session."""
    print("\nTesting profile access...")
    async with httpx.AsyncClient() as client:
        # Use cookies from signin response
        client.cookies = cookies
        response = await client.get(
            f"{BASE_URL}/auth/me",
            headers=HEADERS
        )
        print(f"Profile response: {response.status_code}")
        if response.status_code == 200:
            print("+ Profile access successful")
            print(f"User data: {response.json()}")
            return True
        else:
            print(f"- Profile access failed: {response.text}")
            return False

async def main():
    """Main test function."""
    print("Starting authentication flow test...")

    # Test signup
    signup_success = await test_signup()
    if not signup_success:
        print("Signup failed, stopping test")
        return

    # Test signin
    signin_success, cookies = await test_signin()
    if not signin_success:
        print("Signin failed, stopping test")
        return

    # Test profile access
    if cookies:
        profile_success = await test_get_profile(cookies)
        if not profile_success:
            print("Profile access failed")
            return

    print("\n+ All authentication tests passed!")

if __name__ == "__main__":
    asyncio.run(main())