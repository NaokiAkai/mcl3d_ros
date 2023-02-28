// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#ifndef Grid_hpp
#define Grid_hpp

#include <vector>
#include <DopeVector/DopeVector.hpp>

namespace dope {

	/**
	 * @brief The Grid class describes a D-dimensional grid, containing elements
	 *        of a specified type. Each element could be accessed by its regualr
	 *        index as if it was inside a array-like container. Moreover they
	 *        can be accessed by their index and through iterator.
	 *        It is possible also to extract slices, windows and such from the
	 *        grid using its functions.
	 * @param T             Type of the data to be stored.
	 * @param Dimension     Dimension of the grid.
	 * @param Allocator     Allocator to be used to store the data.
	 * @param Args          Parameter allowing to make specialized grid for
	 *                      known fixed sizes.
	 */
	template < typename T, SizeType Dimension, class Allocator = std::allocator< T > >
	class Grid : public DopeVector< T, Dimension > {
	public:

		////////////////////////////////////////////////////////////////////////
		// TYPEDEFS
		////////////////////////////////////////////////////////////////////////

		typedef typename DopeVector<T, Dimension>::IndexD IndexD;
		typedef std::vector<T, Allocator>	          Data;

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// CONSTRUCTORS
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Default constructor.
		 */
		inline Grid() = default;

		/**
		 *    @brief Initializer contructor.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline explicit Grid(const IndexD &size, const T &default_value = T());

		/**
		 *    @brief Initializer contructor.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x]).
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline explicit Grid(const IndexD &size, const IndexD &order, const T &default_value = T());

		/**
		 *    @brief Initializer contructor.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline explicit Grid(const SizeType size, const T &default_value = T());

		/**
		 *    @brief Initializer contructor.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x])
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline explicit Grid(const SizeType size, const IndexD &order, const T &default_value = T());

		/**
		 *    @brief Copy constructor.
		 */
		inline explicit Grid(const Grid &o);

		/**
		 *    @brief Move constructor.
		 */
		inline explicit Grid(Grid &&) = default;

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// DESTRUCTOR
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Default destructor.
		 */
		virtual inline ~Grid();

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// DATA
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Give access to the first element of the grid.
		 *    @return The const pointer to the first element of the grid.
		 */
		inline const T * data() const;

		/**
		 *    @brief Give access to the first element of the grid.
		 *    @return The pointer to the first element of the grid.
		 */
		inline T * data();

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// CONVERSIONS
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Convert the grid to a std::vector< T, Allocator >.
		 *    @return A const reference to the underneath data of the grid.
		 */
		inline const Data & to_stdvector() const;

		/**
		 *    @brief Convert the grid to a std::vector< T, Allocator >.
		 *    @return A reference to the underneath data of the grid.
		 */
		inline Data & to_stdvector();

		/**
		 *    @brief Cast the grid to a std::vectot< T, Allocator >.
		 *    @return A const reference to the underneath data of the grid.
		 */
		explicit inline operator const Data&() const;

		/**
		 *    @brief Cast the grid to a std::vectot< T, Allocator >.
		 *    @return A reference to the underneath data of the grid.
		 */
		explicit inline operator Data&();



		////////////////////////////////////////////////////////////////////////
		// INFORMATION
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Check the number of elements in the grid.
		 *    @return true if the grid has no elements. false otherwise.
		 */
		inline bool empty() const;

		/**
		 *    @brief Check if a given grid is equal to this.
		 *    @return true if the grids are equal. false otherwise.
		 *    @note Grid sizes must match as well.
		 */
		inline bool operator==(const Grid &o) const;

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// RESET
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Erase all the grid elements, setting it empty.
		 */
		inline void clear();

		/**
		 *    @brief Set all the grid elements to a given value.
		 *    @param default_value      The value all the elements are set to.
		 */
		inline void reset(const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note For performance reasons, data is not kept. See
		 *          conservativeResize.
		 */
		inline void resize(const IndexD &size, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x]).
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note For performance reasons, data is not kept. See
		 *          conservativeResize.
		 */
		inline void resize(const IndexD &size, const IndexD &order, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note For performance reasons, data is not kept. See
		 *          conservativeResize.
		 */
		inline void resize(const SizeType size, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x])
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note For performance reasons, data is not kept. See
		 *          conservativeResize.
		 */
		inline void resize(const SizeType size, const IndexD &order, const T & default_value = T());

		/**
		 *    @brief Resize the container assigning values.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline void assign(const IndexD &size, const T &default_value);

		/**
		 *    @brief Resize the container assigning values.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x]).
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline void assign(const IndexD &size, const IndexD &order, const T &default_value);

		/**
		 *    @brief Resize the container assigning values.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline void assign(const SizeType size, const T &default_value);

		/**
		 *    @brief Resize the container assigning values.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x])
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 */
		inline void assign(const SizeType size, const IndexD &order, const T &default_value);

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note Data is kept as long as possible. That is, the block at the
		 *          origin with minimum size that fits into the new grid is
		 *          copied. See resize for faster resizing.
		 */
		inline void conservativeResize(const IndexD &size, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x]).
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note Data is kept as long as possible. That is, the block at the
		 *          origin with minimum size that fits into the new grid is
		 *          copied. See resize for faster resizing.
		 */
		inline void conservativeResize(const IndexD &size, const IndexD &order, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note Data is kept as long as possible. That is, the block at the
		 *          origin with minimum size that fits into the new grid is
		 *          copied. See resize for faster resizing.
		 */
		inline void conservativeResize(const SizeType size, const T &default_value = T());

		/**
		 *    @brief Resize the container.
		 *    @param size               Sizes of the D-dimensional grid. The
		 *                              grid will be an hypercube.
		 *    @param order              A permutation of the matrix indices that
		 *                              allows the use of different access (e.g.
		 *                              grid[x][y][z] instead of grid[z][y][x])
		 *    @param default_value      Default value assigned to the grid
		 *                              elements.
		 *    @note Data is kept as long as possible. That is, the block at the
		 *          origin with minimum size that fits into the new grid is
		 *          copied. See resize for faster resizing.
		 */
		inline void conservativeResize(const SizeType size, const IndexD &order, const T &default_value = T());

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// ASSIGNMENTS
		////////////////////////////////////////////////////////////////////////

		/**
		 *    @brief Copy assignment operator.
		 */
		inline Grid & operator=(const Grid &o);

		/**
		 *    @brief Move assignment operator.
		 */
		inline Grid & operator=(Grid &&o) = default;

#ifdef DOPE_USE_RTTI
		/**
		 *    @brief Copies all single elements from o to this matrix.
		 *    @param o                  The matrix to copy from.
		 */
		inline void import(const DopeVector<T, Dimension> &o) override;
#endif

		/**
		 *    @brief Swap this with a given grid.
		 *    @note Swap operation is performend in O( 1 ).
		 */
		virtual inline void swap(Grid &o);

		////////////////////////////////////////////////////////////////////////

	protected:
		Data _data;         ///< Elements of the grid.

	private:
		// hyde some methods from DopeVector
		using DopeVector<T, Dimension>::reset;
	};


	/**
	 *     @brief Alias for a grid using the default allocator.
	 */
	template < typename T, SizeType Dimension >
	using StandardGrid = Grid<T, Dimension>;

	/**
	 *     @brief Alias for a 2D grid using the default allocator.
	 */
	template < typename T >
	using StandardGrid2D = Grid<T, static_cast<SizeType>(2)>;

	/**
	 *     @brief Alias for a 3D grid using the default allocator.
	 */
	template < typename T >
	using StandardGrid3D = Grid<T, static_cast<SizeType>(3)>;

}

#include <DopeVector/internal/inlines/Grid.inl>

#endif // Grid_hpp
