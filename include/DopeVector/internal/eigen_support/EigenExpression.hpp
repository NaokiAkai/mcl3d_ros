// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#ifndef EigenExpression_hpp
#define EigenExpression_hpp

#ifdef DOPE_USE_EIGEN

#include <DopeVector/internal/Expression.hpp>
#include <Eigen/Core>

namespace dope {

	namespace internal {

		template < class Derived, class Er, typename T, SizeType Dimension, typename Op >
		class EigenStaticArrayBinaryExpression : public StaticArrayExpression<EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, Op>, T, Dimension> {
			static_assert((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Dimension && Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1) ||
						  (Eigen::MatrixBase<Derived>::RowsAtCompileTime == 1 && Eigen::MatrixBase<Derived>::ColsAtCompileTime == Dimension), "Eigen object must be a vertical vector.");
		private:
			const Eigen::MatrixBase<Derived>                  &_el;
			const Er	                                          &_er;
			mutable std::array<std::function<T()>, Dimension>  _values;
			static const Op                                    _op;

		public:
			inline EigenStaticArrayBinaryExpression(const Eigen::MatrixBase<Derived> &el, const Er &er);

			inline T operator[](const SizeType i) const;
		};



		template < class El, typename T, SizeType Dimension, class Derived, typename Op >
		class StaticArrayBinaryEigenExpression : public StaticArrayExpression<StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, Op>, T, Dimension> {
			static_assert((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Dimension && Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1) ||
						  (Eigen::MatrixBase<Derived>::RowsAtCompileTime == 1 && Eigen::MatrixBase<Derived>::ColsAtCompileTime == Dimension), "Eigen object must be a vertical vector.");
		private:
			const El                                          &_el;
			const Eigen::MatrixBase<Derived>                  &_er;
			mutable std::array<std::function<T()>, Dimension>  _values;
			static const Op                                    _op;

		public:
			inline StaticArrayBinaryEigenExpression(const El &el, const Eigen::MatrixBase<Derived> &er);

			inline T operator[](const SizeType i) const;
		};



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::plus<T>> operator+ (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er);



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::minus<T>> operator- (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er);



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::multiplies<T>> operator* (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er);



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::divides<T>> operator/ (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er);



		template < class Derived, class Er, typename T, SizeType Dimension >
		inline EigenStaticArrayBinaryExpression<Derived, Er, T, Dimension, std::modulus<T>> operator% (const Eigen::MatrixBase<Derived> &el, const StaticArrayExpression<Er, T, Dimension> &er);



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::plus<T>> operator+ (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er);



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::minus<T>> operator- (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er);



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::multiplies<T>> operator* (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er);



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::divides<T>> operator/ (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er);



		template < class El, typename T, SizeType Dimension, class Derived >
		inline StaticArrayBinaryEigenExpression<El, T, Dimension, Derived, std::modulus<T>> operator% (const StaticArrayExpression<El, T, Dimension> &el, const Eigen::MatrixBase<Derived> &er);

	}

}

#include <DopeVector/internal/inlines/eigen_support/EigenExpression.inl>

#endif

#endif
