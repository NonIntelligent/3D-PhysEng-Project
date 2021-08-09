#pragma once

constexpr double PI = 3.141592653589793;
constexpr float PI_f = 3.1415926f;
double toRad(double degrees);
float toRadf(float degrees);
double toDegrees(double radians);
float toDegreesf(float radians);
// Returns an integer based on the sign of the value. (1 = +, 0 = 0, -1 = -).
template<typename T> int getSign(T val) {
	// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
	return (T(0) < val) - (val < T(0));
};

struct Vec3f {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	Vec3f() {
	};

	Vec3f(float x, float y, float z);
	Vec3f(Vec3f* vector);

	~Vec3f() {
	};

	void set(float x, float y, float z);

	// Adds the components of a and b then stores the value in result.
	static void add(Vec3f* result, const Vec3f* a, const Vec3f* b);

	// Subtracts the components of a and b then stores the value in result.
	static void sub(Vec3f* result, const Vec3f* a, const Vec3f* b);

	// Multiplies each component of a by k then stores each value in result.
	static void scale(Vec3f* result, const Vec3f* a, float k);

	// Adds each component of a * k to result.
	static void increment(Vec3f* result, const Vec3f* a, float k);

	// Computes the dot product of a and b.
	static float dot(const Vec3f* a, const Vec3f* b);

	// Computes the cross product of a and b and replaces result.
	static void cross(Vec3f* result, const Vec3f* a, const Vec3f* b);

	// Normalises the vector.
	void normalise();

	// Multiplies each component by -1.
	void negate();

	// Provides the magnitude of the vector squared.
	float lengthSquare() const;

	bool compare(Vec3f &a) const;

};

struct Mat3f {
	float matrix[3][3];

	Mat3f() { setAsIdentity(); };

	Mat3f(const Mat3f* mat);

	~Mat3f() {
	};

	/*
	Replace the data of the matrix at the specified row given an array.
	@param index The row index to replace (0-3)
	*/
	void setRowData(int index, float* array);
	/* Replace entire row data given a vector.
	@overload
	*/
	void setRowData(int index, Vec3f* vec);

	/*
	Replace the data of the matrix at the specified column given an array.
	@param index The column index to replace (0-3)
	*/
	void setColData(int index, float* array);
	/* Replace entire column data given a vector.
	@overload
	*/
	void setColData(int index, Vec3f* vec);

	// Returns the entire row as a vector.
	Vec3f getRowAsVec(int index) const;

	// Returns the entire column as a vector.
	Vec3f getColAsVec(int index) const;

	// Set matrix as identity.
	void setAsIdentity();

	// Set all values to 0.
	void setAsZero();

	// Calculate the determinant of this matrix.
	float determinant() const;

	// Transpose this matrix.
	void transpose();

	// Add each corresponding value of matrix a with b and store in result.
	static void add(Mat3f* result, Mat3f* a, Mat3f* b);

	// Subtract each corresponding value of matrix a with b and store in result.
	static void sub(Mat3f* result, Mat3f* a, Mat3f* b);

	// Scale each value in matrix a by b and store in result.
	static void scale(Mat3f* result, Mat3f* a, float b);

	// Multiply matrix a and b in the order (a * b) and store in result.
	// Result cannot be the same as input b.
	static void mult(Mat3f* result, Mat3f* a, Mat3f* b);

	// Multiply matrix a and vector b in the order (a * b) and store in result.
	// Result cannot be the same as input b.
	static void mult(Vec3f* result, Mat3f* a, Vec3f* b);

	// Calculate the inverse of matrix a and stores in result.
	static bool inverse(Mat3f* result, Mat3f* a);

	// Creates a rotation matrix and multiplies as (rotation * result).
	static void rotate(Mat3f* result, Vec3f axis, float angle);

	// Creates a rotation matrix and multiplies as (rotation * result).
	static void rotate(Mat3f* result, Vec3f axis, Vec3f angles);

};

struct Vec4f {
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
	float w = 1.f;

	Vec4f() {
	};

	Vec4f(float x, float y, float z, float w);
	Vec4f(Vec4f* vector);

	~Vec4f() {
	};

	void set(float x, float y, float z, float w);

	// Adds the components of a and b then stores the value in result.
	static void add(Vec4f* result, Vec4f* a, Vec4f* b);

	// Subtracts the components of a and b then stores the value in result.
	static void sub(Vec4f* result, Vec4f* a, Vec4f* b);

	// Multiplies each component of a by k then stores each value in result.
	static void scale(Vec4f* result, Vec4f* a, float k);

	// Adds each component of a * k to result.
	static void increment(Vec4f* result, Vec4f* a, float k);

	// Computes the dot product of a and b.
	static float dot(Vec4f* a, Vec4f* b);

	// Normalises the vector.
	void normalise();

	// Multiplies each component by -1.
	void negate();

	// Provides the magnitude of the vector squared.
	float lengthSquare() const;

	// Compares the values between both vectors for equality
	bool compare(Vec4f &a) const;
};

struct Mat4f {
	float matrix[4][4];

	Mat4f() { setAsIdentity(); };

	Mat4f(const Mat4f* mat);

	~Mat4f() {
	};

	/*
	Replace the data of the matrix at the specified row given an array.
	@param index The row index to replace (0-3)
	*/
	void setRowData(int index, float* array);
	/* Replace entire row data given a vector.
	@overload
	*/
	void setRowData(int index, Vec4f* vec);
	/* Replace the first 3 values in the row given a vector.
	@overload
	*/
	void setRowData(int index, Vec3f* vec);

	/*
	Replace the data of the matrix at the specified column given an array.
	@param index The column index to replace (0-3)
	*/
	void setColData(int index, float* array);
	/* Replace entire column data given a vector.
	@overload
	*/
	void setColData(int index, Vec4f* vec);
	/* Replace the first 3 column values given a vector.
	@overload
	*/
	void setColData(int index, Vec3f* vec);

	// Returns the first 3 components as row vector.
	Vec3f getRowAsVec3(int index) const;

	// Returns the entire row as a vector.
	Vec4f getRowAsVec4(int index) const;

	// Returns the first 3 components as a column vector.
	Vec3f getColAsVec3(int index) const;	
	// Returns the entire column as a vector.
	Vec4f getColAsVec4(int index) const;

	// Returns a 3x3 matrix that exclude the given row and column.
	Mat3f get3x3Matrix(int row, int col) const;

	// Set matrix as identity.
	void setAsIdentity();

	// Set all values to 0.
	void setAsZero();

	// Calculate the determinant of this matrix.
	float determinant() const;

	// Transpose this matrix.
	void transpose();

	// Copies the data in the matrix into the given 1D array in row-major order.
	void data(float arr[16]) const;

	// Copies the data in the matrix into the given 1D array in col-major order.
	void dataColMaj(float arr[16]) const;

	// Add each corresponding value of matrix a with b and store in result.
	static void add(Mat4f* result, Mat4f* a, Mat4f* b);

	// Subtract each corresponding value of matrix a with b and store in result.
	static void sub(Mat4f* result, Mat4f* a, Mat4f* b);

	// Scale each value in matrix a by b and store in result.
	static void scale(Mat4f* result, Mat4f* a, float b);

	// Multiply matrix a and b in the order (a * b) and store in result.
	// Result cannot be the same as input b.
	static void mult(Mat4f* result, Mat4f* a, Mat4f* b);

	// Multiply matrix a and vector b in the order (a * b) and store in result.
	// Result cannot be the same as input b.
	static void mult(Vec4f* result, Mat4f* a, Vec4f* b);

	// Calculate the inverse of matrix a and stores in result.
	static bool inverse(Mat4f* result, Mat4f* a);

	// Creates a rotation matrix and multiplies as (rotation * result).
	static void rotate(Mat4f* result, Vec3f axis, float angle);

	// Creates a rotation matrix and multiplies as (rotation * result).
	static void rotate(Mat4f* result, Vec3f axis, Vec3f angles);

	// Creates a translation matrix and multiplies as (translation * result).
	static void translate(Mat4f* result, Vec3f translate);

	// Creates a scale matrix and multiplies as (scale * result).
	static void scaleVec(Mat4f* result, Vec3f scalar);
};
