# find . -type f -print0 | xargs -0 dos2unix
find . -type f \( -name "*.h" -o -name "*.c" -o -name "*.cpp" \) -print0 | xargs -0 dos2unix
