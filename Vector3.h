#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

template <typename T>
class Vector3 {
public:
    T x, y, z;

    Vector3();
    Vector3(T x, T y, T z);

    Vector3<T> operator+(const Vector3<T>& other) const;
    Vector3<T> operator-(const Vector3<T>& other) const;
    Vector3<T> operator*(T scalar) const;
    Vector3<T> operator/(T scalar) const;

    T dot(const Vector3<T>& other) const;
    Vector3<T> cross(const Vector3<T>& other) const;

    T magnitude() const;
    Vector3<T> normalize() const;
};

template <typename T>
Vector3<T>::Vector3() : x(0), y(0), z(0) {}

template <typename T>
Vector3<T>::Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

template <typename T>
Vector3<T> Vector3<T>::operator+(const Vector3<T>& other) const {
    return Vector3<T>(x + other.x, y + other.y, z + other.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator-(const Vector3<T>& other) const {
    return Vector3<T>(x - other.x, y - other.y, z - other.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator*(T scalar) const {
    return Vector3<T>(x * scalar, y * scalar, z * scalar);
}

template <typename T>
Vector3<T> Vector3<T>::operator/(T scalar) const {
    return Vector3<T>(x / scalar, y / scalar, z / scalar);
}

template <typename T>
T Vector3<T>::dot(const Vector3<T>& other) const {
    return x * other.x + y * other.y + z * other.z;
}

template <typename T>
Vector3<T> Vector3<T>::cross(const Vector3<T>& other) const {
    return Vector3<T>(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

template <typename T>
T Vector3<T>::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

template <typename T>
Vector3<T> Vector3<T>::normalize() const {
    T mag = magnitude();
    return Vector3<T>(x / mag, y / mag, z / mag);
}

// Explicit template instantiation
template class Vector3<float>;
template class Vector3<double>;
template class Vector3<int>;

#endif // VECTOR3_H
