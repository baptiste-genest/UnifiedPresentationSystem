echo "[slope] creating new slope project $1"

echo "[slope] creating folder in project"
mkdir "projects/$1"

echo "[slope] copying template file at projects/$1/main.cpp"
cp template_project.cpp projects/$1/main.cpp
sed -i "s/slope_PROJECT_NAME/$1/g" projects/$1/main.cpp

echo "[slope] adding project in cmake"
echo "add_executable($1 projects/$1/main.cpp)">>CMakeLists.txt
echo "target_link_libraries($1 slope)">>CMakeLists.txt
