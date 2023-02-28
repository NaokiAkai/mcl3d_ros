// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#ifdef DOPE_USE_EIGEN

#include <DopeVector/internal/eigen_support/EigenExpression.hpp>

namespace dope {

	namespace internal {

		template < class Derived, class Er, typename T, SizeType Dimension, typename Op >
		const Op EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, Op>::_op = Op();

		template < class Derived, class Er, typename T, SizeType Dimension, typename Op >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, Op>::EigenStaticArrayBinaryExpression(const Eigen::MatrixBase<Derived> &el, const Er &er)
		    : _el(el), _er(er)
		{ }

		template < class Derived, class Er, typename T, SizeType Dimension, typename Op >
		inline T EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, Op>::operator[](const SizeType i) const
		{
			if (!_values[i]) {
				T t = _op(_el[i], _er.getAt(i));
				_values[i] = [t]()->T{ return t; };
			}
			return _values[i]();
		}



		template < class El, typename T, SizeType Dimension, class Derived, typename Op >
		const Op StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, Op>::_op = Op();

		template < class El, typename T, SizeType Dimension, class Derived, typename Op >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, Op>::StaticArrayBinaryEigenExpression(const El &el, const Eigen::MatrixBase<Derived> &er)
		    : _el(el), _er(er)
		{ }

		template < class El, typename T, SizeType Dimension, class Derived, typename Op >
		inline T StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, Op>::operator[](const SizeType i) const
		{
			if (!_values[i]) {
				T t = _op(_el.getAt(i), _er[i]);
				_values[i] = [t]()->T{ return t; };
			}
			return _values[i]();
		}



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::plus<T>> operator+ (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::plus<T>>(el, er);
		}



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::minus<T>> operator- (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::minus<T>>(el, er);
		}



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::multiplies<T>> operator* (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::multiplies<T>>(el, er);
		}



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::divides<T>> operator/ (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::divides<T>>(el, er);
		}



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::modulus<T>> operator% (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::modulus<T>>(el, er);
		}



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::plus<T>> operator+ (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er)
		{
			return StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::plus<T>>(el, er);
		}



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::minus<T>> operator- (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er)
		{
			return StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::minus<T>>(el, er);
		}



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::multiplies<T>> operator* (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er)
		{
			return StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::multiplies<T>>(el, er);
		}



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::divides<T>> operator/ (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er)
		{
			return StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::divides<T>>(el, er);
		}



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::modulus<T>> operator% (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er)
		{
			return StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::modulus<T>>(el, er);
		}

	}

}

#endif
