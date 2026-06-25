import os

def main():
    root = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN"
    print(f"Scanning {root} for .trn files...")
    for r, dirs, files in os.walk(root):
        for f in files:
            if f.endswith(".trn"):
                full_path = os.path.join(r, f)
                print(f"Found: {full_path} (size={os.path.getsize(full_path)} bytes)")

if __name__ == "__main__":
    main()
