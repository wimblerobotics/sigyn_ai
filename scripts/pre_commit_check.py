#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Sigyn AI Contributors
"""
Pre-commit validation script for Sigyn AI.

Checks for common issues before committing:
- No large files (datasets, models)
- No API keys
- Code style (if black/flake8 installed)
"""

import sys
from pathlib import Path
import re

# ANSI colors
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'

def check_file_size(max_mb=10):
    """Check for large files that shouldn't be committed."""
    print("🔍 Checking for large files...")
    
    large_files = []
    for file in Path('.').rglob('*'):
        if file.is_file() and not any(p in file.parts for p in ['.git', '__pycache__', 'venv', 'env']):
            size_mb = file.stat().st_size / (1024 * 1024)
            if size_mb > max_mb:
                large_files.append((file, size_mb))
    
    if large_files:
        print(f"{RED}❌ Large files detected (>{max_mb}MB):{RESET}")
        for file, size in large_files:
            print(f"   {file}: {size:.1f}MB")
        return False
    
    print(f"{GREEN}✅ No large files{RESET}")
    return True


def check_api_keys():
    """Check for API keys in files."""
    print("\n🔍 Checking for API keys...")
    
    patterns = [
        r'ROBOFLOW_API_KEY\s*=\s*["\'][^"\']+["\']',
        r'api_key\s*=\s*["\'][A-Za-z0-9]{20,}["\']',
        r'AWS_SECRET_ACCESS_KEY',
        r'GITHUB_TOKEN',
    ]
    
    suspicious_files = []
    for file in Path('.').rglob('*.py'):
        if '.git' in file.parts or '__pycache__' in file.parts:
            continue
        
        try:
            content = file.read_text()
            for pattern in patterns:
                if re.search(pattern, content):
                    # Exception: if it's an example/placeholder
                    if 'your_key_here' in content.lower() or 'example' in str(file):
                        continue
                    suspicious_files.append((file, pattern))
        except:
            pass
    
    if suspicious_files:
        print(f"{RED}❌ Possible API keys found:{RESET}")
        for file, pattern in suspicious_files:
            print(f"   {file}: {pattern}")
        print(f"{YELLOW}   Review these files carefully!{RESET}")
        return False
    
    print(f"{GREEN}✅ No API keys detected{RESET}")
    return True


def check_gitignore():
    """Check that important patterns are in .gitignore."""
    print("\n🔍 Checking .gitignore...")
    
    required_patterns = [
        'datasets/',
        'models/',
        '*.pt',
        '*.hef',
        '*.blob',
        '*.engine',
        '.env',
        '__pycache__/',
    ]
    
    try:
        gitignore = Path('.gitignore').read_text()
        missing = [p for p in required_patterns if p not in gitignore]
        
        if missing:
            print(f"{YELLOW}⚠️  Missing .gitignore patterns:{RESET}")
            for pattern in missing:
                print(f"   {pattern}")
            return False
        
        print(f"{GREEN}✅ .gitignore is complete{RESET}")
        return True
    except FileNotFoundError:
        print(f"{RED}❌ .gitignore not found!{RESET}")
        return False


def check_code_style():
    """Run black and flake8 if available."""
    print("\n🔍 Checking code style...")
    
    try:
        import subprocess
        
        # Check black
        result = subprocess.run(['black', '--check', 'src/'], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"{YELLOW}⚠️  Code not formatted with black{RESET}")
            print(f"   Run: black src/")
            return False
        
        print(f"{GREEN}✅ Code style is good{RESET}")
        return True
        
    except FileNotFoundError:
        print(f"{YELLOW}⚠️  black not installed (optional){RESET}")
        print(f"   Install: pip install black")
        return True  # Don't fail if not installed


def main():
    print("=" * 60)
    print("🔬 Sigyn AI Pre-Commit Validation")
    print("=" * 60)
    
    checks = [
        check_gitignore(),
        check_file_size(),
        check_api_keys(),
        check_code_style(),
    ]
    
    print("\n" + "=" * 60)
    if all(checks):
        print(f"{GREEN}✅ All checks passed! Safe to commit.{RESET}")
        return 0
    else:
        print(f"{RED}❌ Some checks failed. Review and fix before committing.{RESET}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
