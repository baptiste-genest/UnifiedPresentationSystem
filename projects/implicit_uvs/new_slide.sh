echo "[slope] add cpp folder in project"
touch "slides/$1.cpp"

echo "[slope] add declaration in header"
echo "void CreateSlides$1(slope::Slideshow&);" >> implicit_uvs_slides.h
cp slide_template.cpp slides/$1.cpp
sed -i "s/NAME/$1/g" slides/$1.cpp
