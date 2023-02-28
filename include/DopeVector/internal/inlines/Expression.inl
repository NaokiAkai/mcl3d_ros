// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#include <DopeVector/internal/Expression.hpp>

namespace dope {

	namespace internal {

		template < class E, typename T, SizeType Dimension >
		inline T StaticArrayExpression<E, T, Dimension>::getAt(const SizeType i) const
		{
			return static_cast<E const&>(*this)[i];
		}

		template < class E, typename T, SizeType Dimension >
		inline StaticArrayExpression<E, T, Dimension>::operator E const&() const
		{
			return static_cast<const E &>(*this);
		}

		template < class E, typename T, SizeType Dimension >
		inline StaticArrayExpression<E, T, Dimension>::operator E &()
		{
			return static_cast<E &>(*this);
		}



		template < class E, typename T, SizeType Dimension, typename Op >
		const Op StaticArrayUnaryExpression<E, T, Dimension, Op>::_op = Op();

		template < class E, typename T, SizeType Dimension, typename Op >
		inline StaticArrayUnaryExpression<E, T, Dimension, Op>::StaticArrayUnaryExpression(const E &e)
		    : _e(e)
		{ }

		template < class E, typename T, SizeType Dimension, typename Op >
		inline T StaticArrayUnaryExpression<E, T, Dimension, Op>::operator[](const SizeType i) const
		{
			if (!_values[i]) {
				T t = _op(_e.getAt(i));
				_values[i] = [t]()->T{ return t; };
			}
			return _values[i]();
		}



		template < class El, class Er, typename T, SizeType Dimension, typename Op >
		const Op StaticArrayBinaryExpression<El, Er, T, Dimension, Op>::_op = Op();

		template < class El, class Er, typename T, SizeType Dimension, typename Op >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, Op>::StaticArrayBinaryExpression(const El &el, const Er &er)
		    : _el(el), _er(er)
		{ }

		template < class El, class Er, typename T, SizeType Dimension, typename Op >
		inline T StaticArrayBinaryExpression<El, Er, T, Dimension, Op>::operator[](const SizeType i) const
		{
			if (!_values[i]) {
				T t = _op(_el.getAt(i), _er.getAt(i));
				_values[i] = [t]()->T{ return t; };
			}
			return _values[i]();
		}


		template < class E, typename T, SizeType Dimension >
		inline StaticArrayUnaryExpression<E, T, Dimension, std::negate<T>> operator- (const StaticArrayExpression<E, T, Dimension> &e)
		{
			return StaticArrayUnaryExpression<E, T, Dimension, std::negate<T>>(e);
		}



		template < class El, class Er, typename T, SizeType Dimension >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, std::plus<T>> operator+ (const StaticArrayExpression<El, T, Dimension> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return StaticArrayBinaryExpression<El, Er, T, Dimension, std::plus<T>>(el, er);
		}



		template < class El, class Er, typename T, SizeType Dimension >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, std::minus<T>> operator- (const StaticArrayExpression<El, T, Dimension> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return StaticArrayBinaryExpression<El, Er, T, Dimension, std::minus<T>>(el, er);
		}



		template < class El, class Er, typename T, SizeType Dimension >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, std::multiplies<T>> operator* (const StaticArrayExpression<El, T, Dimension> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return StaticArrayBinaryExpression<El, Er, T, Dimension, std::multiplies<T>>(el, er);
		}



		template < class El, class Er, typename T, SizeType Dimension >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, std::divides<T>> operator/ (const StaticArrayExpression<El, T, Dimension> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return StaticArrayBinaryExpression<El, Er, T, Dimension, std::divides<T>>(el, er);
		}



		template < class El, class Er, typename T, SizeType Dimension >
		inline StaticArrayBinaryExpression<El, Er, T, Dimension, std::modulus<T>> operator% (const StaticArrayExpression<El, T, Dimension> &el, const StaticArrayExpression<Er, T, Dimension> &er)
		{
			return StaticArrayBinaryExpression<El, Er, T, Dimension, std::modulus<T>>(el, er);
		}

	}
	
}
