// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#include <DopeVector/Index.hpp>
#include <stdexcept>
#include <algorithm>

namespace dope {

	template < SizeType Dimension > template < typename ... Sizes, class >
	inline Index<Dimension>::Index(const SizeType size0,  Sizes &&...sizes)
	    : std::array<SizeType, Dimension>({{size0, std::forward<SizeType>(sizes)...}})
	{ }

	template < SizeType Dimension >
	inline Index<Dimension>::Index(const std::initializer_list<SizeType> &il)
	{
		std::copy(il.begin(), il.begin() + std::min(il.size(), Dimension), std::array<SizeType, Dimension>::begin());
		std::fill(std::array<SizeType, Dimension>::begin() + std::min(il.size(), Dimension), std::array<SizeType, Dimension>::end(), std::array<SizeType, Dimension>::operator[](std::min(Dimension, std::min(il.size(), Dimension)) - 1));
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>::Index(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) = e.getAt(i);
	}

#ifdef DOPE_USE_EIGEN
	template < SizeType Dimension > template < class Derived >
	inline Index<Dimension>::Index(const Eigen::MatrixBase<Derived> &e)
	{
		static_assert((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Dimension && Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1) ||
					  (Eigen::MatrixBase<Derived>::RowsAtCompileTime == 1 && Eigen::MatrixBase<Derived>::ColsAtCompileTime == Dimension), "Eigen object must be a vertical vector.");
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) = e[i];
	}
#endif

	template < SizeType Dimension >
	inline Index<Dimension> & Index<Dimension>::operator= (const std::initializer_list<SizeType> &il)
	{
		std::copy(il.begin(), il.begin() + std::min(il.size(), Dimension), std::array<SizeType, Dimension>::begin());
		std::fill(std::array<SizeType, Dimension>::begin() + std::min(il.size(), Dimension), std::array<SizeType, Dimension>::end(), std::array<SizeType, Dimension>::operator[](std::min(Dimension, std::min(il.size(), Dimension)) - 1));
		return *this;
	}

	template < SizeType Dimension >
	inline bool Index<Dimension>::isApprox(const Index<Dimension> &o) const
	{
		return *this == o;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) = e.getAt(i);
		return *this;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator+=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) += e.getAt(i);
		return *this;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator-=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) -= e.getAt(i);
		return *this;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator*=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) *= e.getAt(i);
		return *this;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator/=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) /= e.getAt(i);
		return *this;
	}

	template < SizeType Dimension > template < class E >
	inline Index<Dimension>& Index<Dimension>::operator%=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e)
	{
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) %= e.getAt(i);
		return *this;
	}

#ifdef DOPE_USE_EIGEN
	/**
	 * @brief Eigen expression assignment operator.
	 */
	template < SizeType Dimension > template < class Derived >
	inline Index<Dimension>& Index<Dimension>::operator=(const Eigen::MatrixBase<Derived> &e)
	{
		static_assert((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Dimension && Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1) ||
					  (Eigen::MatrixBase<Derived>::RowsAtCompileTime == 1 && Eigen::MatrixBase<Derived>::ColsAtCompileTime == Dimension), "Eigen object must be a vertical vector.");
		for (SizeType i = 0; i < Dimension; ++i)
			std::array<SizeType, Dimension>::operator[](i) = e[i];
		return *this;
	}
#endif

	template < SizeType Dimension >
	inline SizeType Index<Dimension>::sum() const
	{
		SizeType sum = static_cast<SizeType>(0);
		for (SizeType i = static_cast<SizeType>(0); i < Dimension; ++i)
			sum += std::array<SizeType, Dimension>::at(i);
		return sum;
	}

	template < SizeType Dimension >
	inline SizeType Index<Dimension>::prod() const
	{
		SizeType prod = static_cast<SizeType>(1);
		for (SizeType i = static_cast<SizeType>(0); i < Dimension; ++i)
			prod *= std::array<SizeType, Dimension>::at(i);
		return prod;
	}

	template < SizeType Dimension >
	inline constexpr Index<Dimension> Index<Dimension>::Zero()
	{
		return Index({{0}});
	}

	template < SizeType Dimension >
	inline constexpr Index<Dimension> Index<Dimension>::Ones()
	{
		return Index({{1}});
	}

	template < SizeType Dimension >
	inline constexpr Index<Dimension> Index<Dimension>::Constant( const SizeType value )
	{
		return Index({{value}});
	}

	template < SizeType Dimension >
	inline SizeType to_position(const Index<Dimension> &index, const Index<Dimension> &range)
	{
		SizeType result = static_cast<SizeType>(0);
		SizeType dimProd = static_cast<SizeType>(1);
		for(SizeType D = Dimension; D > static_cast<SizeType>(0); --D) {
			SizeType d = D - static_cast<SizeType>(1);
			result += index[d] * dimProd;
			dimProd *= range[d];
		}
		return result;
	}

	template < SizeType Dimension >
	inline SizeType to_positionFromOffset(const Index<Dimension> &index, const Index<Dimension> &offset)
	{
		SizeType position = static_cast<SizeType>(0);
		for(SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			position += index[d] * offset[d];
		return position;
	}

	template < SizeType Dimension >
	inline Index<Dimension> to_index(const SizeType position, const Index<Dimension> &range)
	{
		Index<Dimension> result = Index<Dimension>::Zero();
		SizeType i = position;
		for(SizeType D = Dimension; D > static_cast<SizeType>(0); --D) {
			SizeType d = D - static_cast<SizeType>(1);
			if (range[d] == static_cast<SizeType>(0))
				throw std::overflow_error("Divide by zero exception");
			result[d] = i % range[d];
			i = i / range[d];
		}
		return result;
	}

	template < SizeType Dimension >
	inline Index<Dimension> to_indexFromOffset(const SizeType position, const Index<Dimension> &offset)
	{
		SizeType linear_position = position;
		Index<Dimension> order;
		for (SizeType i = static_cast<SizeType>(0); i < Dimension; ++i)
			order[i] = i;
		std::sort(order.begin(), order.end(), [&offset](const SizeType &l, const SizeType &r)->bool{
			return offset[l] < offset[r];
		});
		Index<Dimension> result = Index<Dimension>::Zero();
		for(SizeType D = Dimension; D > static_cast<SizeType>(0); --D) {
			SizeType d = D - static_cast<SizeType>(1);
			result[order[d]] = linear_position / offset[order[d]];
			linear_position -= result[order[d]] * offset[order[d]];
		}
		return result;
	}

}
