typedef struct image_rep *Image;

Image newImage(int width, int height, char* dataLink);
Image newBackground(char* dataLink);
void freeImage(Image img);
void display_image(Image img, int posx1, int posy1);
void display_clrimg(Image img, Image background);
