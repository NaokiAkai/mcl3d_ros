// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#ifndef Index_hpp
#define Index_hpp

#include <DopeVector/internal/Expression.hpp>
#include <array>

namespace dope {

	namespace internal {

		template < typename T, typename ... Rest >
		struct SizeTypesCheck {
			static constexpr bool value = (std::is_arithmetic<T>::value) && (SizeTypesCheck<Rest...>::value);
		};

		template < typename T >
		struct SizeTypesCheck<T> {
			static constexpr bool value = std::is_arithmetic<T>::value;
		};

	}

	/**
	 * @brief The Index class defines a D-dimensional index, used to refer to an
	 *        element contained in a DopeVector.
	 */
	template < SizeType Dimension >
	class Index : public std::array<SizeType, Dimension>, public internal::StaticArrayExpression<Index<Dimension>, SizeType, Dimension> {
	public:

		////////////////////////////////////////////////////////////////////////
		// CONSTRUCTORS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Default constructor.
		 */
		explicit Index() = default;

		/**
		 * @brief Copy constructor.
		 */
		explicit Index(const Index &) = default;

		/**
		 * @brief Move constructor.
		 */
		explicit Index(Index &&) = default;

		/**
		 * @brief Initializer constructor.
		 * @param size0     The first component of the index.
		 * @param sizes     The remaining components of the index,
		 *                  if its dimension is greater than one.
		 */
		template < typename ... Sizes, class = typename std::enable_if<internal::SizeTypesCheck<SizeType, Sizes...>::value>::type >
		explicit inline Index(const SizeType size0, Sizes &&...sizes);

		/**
		 * @brief Initializer constructor.
		 * @param il        A list of components.
		 */
		inline Index(const std::initializer_list<SizeType> &il);

		/**
		 * @brief Initializer constructor.
		 * @param e         The result of an expression.
		 */
		template < class E >
		inline Index(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

#ifdef DOPE_USE_EIGEN
		/**
		 * @brief Initializer constructor.
		 * @param e         The result of an Eigen expression.
		 */
		template < class Derived >
		inline Index(const Eigen::MatrixBase<Derived> &e);
#endif

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// ASSIGNMENTS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Copy assignment operator.
		 */
		Index        & operator= (const Index &) = default;

		/**
		 * @brief Move assignment operator.
		 */
		Index        & operator= (Index &&) = default;

		/**
		 * @brief List assignment operator.
		 */
		inline Index & operator= (const std::initializer_list<SizeType> &il);

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// RELATIONAL OPERATORS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Returns true if a given index is equal to \p this.
		 * @param o     A index.
		 * @return \p true if the components of the given index are equal to
		 *         the components of \p this. \p false otherwise.
		 * @note This function was made for maintaining compatibility between
		 *       Eigen indices and built-in indices.
		 */
		inline bool isApprox(const Index &o) const;

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// EXPRESSION OPERATORS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Expression assignment operator.
		 */
		template < class E >
		inline Index& operator=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

		/**
		 * @brief Expression addition operator.
		 */
		template < class E >
		inline Index& operator+=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

		/**
		 * @brief Expression difference operator.
		 */
		template < class E >
		inline Index& operator-=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

		/**
		 * @brief Expression product operator.
		 * @warning This operator is not Eigen compatible. In case of using Eigen, please use
		 *          cwiseProduct.
		 */
		template < class E >
		inline Index& operator*=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

		/**
		 * @brief Expression quotient operator.
		 * @warning This operator is not Eigen compatible. In case of using Eigen, please use
		 *          cwiseQuotient.
		 */
		template < class E >
		inline Index& operator/=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

		/**
		 * @brief Expression modulus operator.
		 * @warning This operator is not Eigen compatible.
		 */
		template < class E >
		inline Index& operator%=(const internal::StaticArrayExpression<E, SizeType, Dimension> &e);

#ifdef DOPE_USE_EIGEN
		/**
		 * @brief Eigen expression assignment operator.
		 */
		template < class Derived >
		inline Index& operator=(const Eigen::MatrixBase<Derived> &e);
#endif

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// ASSIGNMENTS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Performs the sum of all the components of \p this.
		 * @return The sum of the components of \p this.
		 */
		inline SizeType sum() const;

		/**
		 * @brief Performs the product of all the components of \p this.
		 * @return The product of the components of \p this.
		 */
		inline SizeType prod() const;

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// ASSIGNMENTS
		////////////////////////////////////////////////////////////////////////

		/**
		 * @brief Returns the index {0, ..., 0}.
		 * @return A index contaning only zeros.
		 * @note This function was made for maintaining compatibility between
		 *       Eigen indices and built-in indices.
		 */
		static inline constexpr Index Zero();

		/**
		 * @brief Returns the index {1, ..., 1}.
		 * @return A index containing only ones.
		 * @note This function was made for maintaining compatibility between
		 *       Eigen indices and built-in indices.
		 */
		static inline constexpr Index Ones();

		/**
		 * @brief Returns the index {value, ... , value}.
		 * @param value     The value each component is set to.
		 * @return A index with all entries equal to the given value.
		 * @note This function was made for maintaining compatibility between
		 *       Eigen indices and built-in indices.
		 */
		static inline constexpr Index Constant(const SizeType value);

		////////////////////////////////////////////////////////////////////////

	};

	////////////////////////////////////////////////////////////////////////
	// TYPEDEFS
	////////////////////////////////////////////////////////////////////////

	typedef Index<1>    Index1;     ///< Alias for 1-dimensional indices
	typedef Index<2>    Index2;     ///< Alias for 2-dimensional indices
	typedef Index<3>    Index3;     ///< Alias for 3-dimensional indices
	typedef Index<4>    Index4;     ///< Alias for 4-dimensional indices

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// UTILITY FUNCTIONS
	////////////////////////////////////////////////////////////////////////

	/**
	 * @brief Returns the linearized index value, according to the given
	 *        range.
	 * @param index     The index to be linearized.
	 * @param range     The range of the linearization.
	 * @return An unsigned integer value defining the linear version of
	 *         the given index.
	 * @note This function was kept outside the Index class for maintaining
	 *       compatibility between Eigen indices and built-in indices.
	 */
	template < SizeType Dimension >
	static inline SizeType to_position(const Index<Dimension> &index, const Index<Dimension> &range);

	/**
	 * @brief Returns the linearized index value, according to the given offset.
	 * @param index         The index to be linearized.
	 * @param offset        The offset in each dimension.
	 * @return An unsigned integer value defining the linear version of
	 *         the given index.
	 * @note This function was kept outside the Index class for maintaining
	 *       compatibility between Eigen indices and built-in indices.
	 */
	template < SizeType Dimension >
	static inline SizeType to_positionFromOffset(const Index<Dimension> &index, const Index<Dimension> &offset);

	/**
	 * @brief Returns the index form of a linear position, according to the given
	 *        range.
	 * @param position      The position to be put in index form.
	 * @param range         The range of the index.
	 * @return An index built from the given position.
	 * @note This function was kept outside the Index class for maintaining
	 *       compatibility between Eigen indices and built-in indices.
	 */
	template < SizeType Dimension >
	static inline Index<Dimension> to_index(const SizeType position, const Index<Dimension> &range);

	/**
	 * @brief Returns the index form of a linear position, according to the given
	 *        offset.
	 * @param position      The position to be put in index form.
	 * @param offset        The offset in each dimension.
	 * @return An index built from the given position.
	 * @note This function was kept outside the Index class for maintaining
	 *       compatibility between Eigen indices and built-in indices.
	 */
	template < SizeType Dimension >
	static inline Index<Dimension> to_indexFromOffset(const SizeType position, const Index<Dimension> &offset);

	////////////////////////////////////////////////////////////////////////

}

#include <DopeVector/internal/inlines/Index.inl>

#endif // Index_hpp
