#ifndef __Moca_CAMERA__
#define __Moca_CAMERA__
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective


const float PI = 3.14159265;
const float PI_OVER_TWO = 1.5707963267948966192313216916397514420985;

inline float Radians(float deg) {
	return deg * (PI / 180.0);
}

class Camera {
public:
	glm::vec3 focus;
	glm::vec2 angle;  // yaw : x,  pitch : y
	float radius;
	float apertureRadius;
	float focalDistance;
	glm::vec2 resolution;
	glm::vec2 fov;

public:
	Camera() {
		angle = glm::vec2(0.0f);
		focus = glm::vec3(0.0f);
		radius = 20.0f;
		apertureRadius = 0.01;
		focalDistance = 20.0f;

		resolution = glm::vec2(0.0f);
		fov = glm::vec2(45.0f, 45.0f);
	}

	glm::mat4 Mat(void) const {
		glm::mat4 rot = glm::mat4(1.0f);
		rot = glm::rotate(rot, Radians(angle.y), glm::vec3(0.0f, 1.0f, 0.0f));
		rot = glm::rotate(rot, Radians(angle.x), glm::vec3(1.0f, 0.0f, 0.0f));

		glm::vec3 dir = glm::vec3(rot[2][0], rot[2][1], rot[2][2]);

		glm::vec3 origin = focus + dir * radius;

		return glm::translate(glm::transpose(rot), -origin);
	}

	glm::vec3 GetPos(void) const {
		glm::mat4 rot = glm::mat4(1.0f);
		rot = glm::rotate(rot, Radians(angle.y), glm::vec3(0.0f, 1.0f, 0.0f));
		rot = glm::rotate(rot, Radians(angle.x), glm::vec3(1.0f, 0.0f, 0.0f));

		glm::vec3 dir = glm::vec3(rot[2][0], rot[2][1], rot[2][2]);
		return -(focus + dir * radius);
	}

	glm::vec3 GetDir(void) const {
		glm::mat4 rot = glm::mat4(1.0f);
		rot = glm::rotate(rot, Radians(angle.y), glm::vec3(0.0f, 1.0f, 0.0f));
		rot = glm::rotate(rot, Radians(angle.x), glm::vec3(1.0f, 0.0f, 0.0f));

		return glm::vec3(rot[2][0], rot[2][1], rot[2][2]);

		//float yaw = Radians(angle.x);
		//float pitch = Radians(angle.y);
		//return glm::vec3(sin(yaw)*cos(pitch), sin(pitch), cos(yaw)*sin(pitch));
	}

	void Zoom(float dz) {
		radius = radius - dz;
		if (radius < 0) {
			radius = 0;
		}
	}

	void Rot(float dx, float dy) {
		angle.x += dx;
		angle.y += dy;

		if (angle.x < -90) {
			angle.x = -90;
		}
		else if (angle.x > 90) {
			angle.x = 90;
		}
	}

	void Move(float dx, float dy) {
		glm::mat4 rot = glm::mat4(1.0f);
		rot = glm::rotate(rot, Radians(angle.y), glm::vec3(0.0f, 1.0f, 0.0f));
		rot = glm::rotate(rot, Radians(angle.x), glm::vec3(1.0f, 0.0f, 0.0f));

		focus += glm::vec3(rot[0][0], rot[0][1], rot[0][2]) * dx;
		focus += glm::vec3(rot[1][0], rot[1][1], rot[1][2]) * dy;
	}

	void SetResolution(float w, float h) {
		resolution = glm::vec2(w, h);
	}

	void ChangeAperture(float a) {
		apertureRadius += (apertureRadius + 0.01) * a; // Change proportional to current apertureRadius.
		float minApertureRadius = 0.0f;
		float maxApertureRadius = 25.0f;
		apertureRadius = glm::clamp(apertureRadius, minApertureRadius, maxApertureRadius);
	}

	void ChangeFocalDistance(float f) {
		focalDistance += f;
		float minFocalDist = 0.2;
		float maxFocalDist = 100.0;
		focalDistance = glm::clamp(focalDistance, minFocalDist, maxFocalDist);
	}
};

#endif