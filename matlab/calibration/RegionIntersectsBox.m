function result = RegionIntersectsBox(PixelList, Box)

xix = [2 4];
yix = [1 3];
xmin = min(Box([xix]));
xmax = max(Box([xix]));
ymin = min(Box([yix]));
ymax = max(Box([yix]));

result = 0;

[npix, ~] = size(PixelList);

for px = 1 : npix,
    x = PixelList(px, 1);  y = PixelList(px, 2);
    if (x >= xmin) && (x <= xmax) && (y >= ymin) && (y <= ymax),
        result = 1;
        return;
    end
end