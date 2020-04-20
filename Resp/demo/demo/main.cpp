#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include <algorithm>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,   255, 0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(p0.x-p1.x)<std::abs(p0.y-p1.y)) {
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x>p1.x) {
        std::swap(p0, p1);
    }

    for (int x=p0.x; x<=p1.x; x++) {
        float t = (x-p0.x)/(float)(p1.x-p0.x);
        int y = p0.y*(1.-t) + p1.y*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
	Vec3f s[2];
	for (int i = 2; i--; ) {
		s[i][0] = C[i] - A[i];
		s[i][1] = B[i] - A[i];
		s[i][2] = A[i] - P[i];
	}
	Vec3f u = cross(s[0], s[1]);
	if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
		return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
	return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color) {
	Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
			bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
		}
	}
	Vec3f P;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			P.z = 0;
			for (int i = 0; i < 3; i++) P.z += pts[i][2] * bc_screen[i];
			if (zbuffer[int(P.x + P.y*width)] < P.z) {
				zbuffer[int(P.x + P.y*width)] = P.z;
				image.set(P.x, P.y, color);
			}
		}
	}
}

Vec3f world2screen(Vec3f v) {
	return Vec3f(int((v.x + 1.)*width / 2. + .5), int((v.y + 1.)*height / 2. + .5), v.z);
}

Vec3f barycentric(Vec2i *pts, Vec2i P)
{
	/*
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts, P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			image.set(P.x, P.y, color);
		}
	}
	*/

	//	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
	Vec3f p1(pts[2][0] - pts[0][0], pts[1][0] - pts[0][0], pts[0][0] - P[0]);
	Vec3f p2(pts[2][1] - pts[0][1], pts[1][1] - pts[0][1], pts[0][1] - P[1]);
	Vec3f u = cross(p1, p2);

	float temp1 = pts[0][0];
	float temp2 = P[0];

	std::cout << "pts[0][0] = " << pts[0][0] << ", P[0] = " << P[0] << ", pts[0][0] - P[0] = "
		<< pts[0][0] - P[0] << std::endl;

	//std::cout << "p1 = " << p1 << ", p2 = " << p2 << ", u = " << u << std::endl;

	/* `pts` and `P` has integer value as coordinates
	   so `abs(u[2])` < 1 means `u[2]` is 0, that means
	   triangle is degenerate, in this case return something with negative coordinates */
	if (std::abs(u[2]) < 1) return Vec3f(-1, 1, 1);
	return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color)
{
	Vec2i bboxmin(image.get_width() - 1, image.get_height() - 1);
	Vec2i bboxmax(0, 0);
	Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			//x的最小值，y的最小值
			bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));

			//x的最大值，y的最大值
			bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
		}
	}


	std::cout << "bboxmin : " << bboxmin << std::endl;
	std::cout << "bboxmax : " << bboxmax << std::endl;

	Vec2i P;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts, P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			image.set(P.x, P.y, color);
		}
	}
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
	if (t0.y == t1.y && t0.y == t2.y) return; // i dont care about degenerate triangles
	if (t0.y > t1.y) std::swap(t0, t1);
	if (t0.y > t2.y) std::swap(t0, t2);
	if (t1.y > t2.y) std::swap(t1, t2);
	int total_height = t2.y - t0.y;
	for (int i = 0; i < total_height; i++) {
		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
		float alpha = (float)i / total_height;
		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
		Vec2i A = t0 + (t2 - t0)*alpha;
		Vec2i B = second_half ? t1 + (t2 - t1)*beta : t0 + (t1 - t0)*beta;
		if (A.x > B.x) std::swap(A, B);
		for (int j = A.x; j <= B.x; j++) {
			image.set(j, t0.y + i, color); // attention, due to int casts t0.y+i != A.y
		}
	}
}

int main(int argc, char** argv) {
	if (2 == argc) {
		model = new Model(argv[1]);
	}
	else {
		model = new Model("obj/african_head/african_head.obj");
	}

	TGAImage image(width, height, TGAImage::RGB);
	Vec3f light_dir(0, 0, -1);
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		Vec2i screen_coords[3];
		Vec3f world_coords[3];
		for (int j = 0; j < 3; j++) {
			Vec3f v = model->vert(face[j]);
			screen_coords[j] = Vec2i((v.x + 1.)*width / 2., (v.y + 1.)*height / 2.);
			world_coords[j] = v;
		}
		Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
		n.normalize();
		float intensity = n * light_dir;
		if (intensity > 0) {
			triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
		}
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	delete model;
	return 0;
}

//void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//	// sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!) 
//	if (t0.y > t1.y) std::swap(t0, t1);
//	if (t0.y > t2.y) std::swap(t0, t2);
//	if (t1.y > t2.y) std::swap(t1, t2);
//	int total_height = t2.y - t0.y;
//
//	//根据相似三角形，求出与纵坐标为y的t1t2、t0t2线段交点A、B，把A、B之间的点设置颜色
//	for (int y = t0.y; y <= t1.y; y++) {
//		int segment_height = t1.y - t0.y + 1;
//		float alpha = (float)(y - t0.y) / total_height;
//		float beta = (float)(y - t0.y) / segment_height; // be careful with divisions by zero 
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = t0 + (t1 - t0)*beta;
//		if (A.x > B.x) std::swap(A, B);
//		for (int j = A.x; j <= B.x; j++) {
//			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
//		}
//	}
//
//	//根据相似梯形，求出与纵坐标为y的t1t2、t0t2线段交点A、B，把A、B之间的点设置颜色
//	for (int y = t1.y; y <= t2.y; y++) {
//		int segment_height = t2.y - t1.y + 1;
//		float alpha = (float)(y - t0.y) / total_height;
//		float beta = (float)(y - t1.y) / segment_height; // be careful with divisions by zero 
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = t1 + (t2 - t1)*beta;
//		if (A.x > B.x) std::swap(A, B);
//		for (int j = A.x; j <= B.x; j++) {
//			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
//		}
//	}
//}

//void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//	// sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!) 
//	if (t0.y > t1.y) std::swap(t0, t1);
//	if (t0.y > t2.y) std::swap(t0, t2);
//	if (t1.y > t2.y) std::swap(t1, t2);
//	int total_height = t2.y - t0.y;
//	for (int y = t0.y; y <= t1.y; y++) {
//		int segment_height = t1.y - t0.y + 1;
//		float alpha = (float)(y - t0.y) / total_height;
//		float beta = (float)(y - t0.y) / segment_height; // be careful with divisions by zero 
//
//		//
//		Vec2i A = t0 + (t2 - t0)*alpha;
//		Vec2i B = t0 + (t1 - t0)*beta;
//		/*image.set(A.x, y, red);
//		image.set(B.x, y, green);*/
//
//		image.set(A.x, A.y, red);
//		image.set(B.x, B.y, green);
//	}
//}

//int main(int argc, char** argv) 
//{
//	TGAImage image(width, height, TGAImage::RGB);
//
//	Vec2i t0[3] = { Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80) };
//	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
//	Vec2i t2[3] = { Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180) };
//	//triangle(t0[0], t0[1], t0[2], image, red);
//	//triangle(t1[0], t1[1], t1[2], image, white);
//	//triangle(t2[0], t2[1], t2[2], image, green);
//
//	triangle(t1, image, red);
//
//	/*
//		Vec3f world_coords[3];
//		Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);*/
//
//    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
//    image.write_tga_file("output.tga");
//    delete model;
//    return 0;
//}


