// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#include <DopeVector/Grid.hpp>

namespace dope {

	////////////////////////////////////////////////////////////////////////////
	// CONSTRUCTORS
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::Grid(const IndexD &size, const T &default_value)
	    : _data(size.prod(), default_value)
	{
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::Grid(const IndexD &size, const IndexD &order, const T &default_value)
	    : _data(size.prod(), default_value)
	{
		IndexD original_offset;
		original_offset[Dimension-1] = 1;
		for (SizeType d = Dimension-1; d > static_cast<SizeType>(0); --d)
			original_offset[d-1] = size[d] * original_offset[d];
		IndexD offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			offset[d] = original_offset[order[d]];
		}
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size, offset);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::Grid(const SizeType size, const T &default_value)
	    : _data(Index<Dimension>::Constant(size).prod(), default_value)
	{
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), Index<Dimension>::Constant(size));
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::Grid(const SizeType size, const IndexD &order, const T &default_value)
	    : _data(Index<Dimension>::Constant(size).prod(), default_value)
	{
		IndexD original_offset;
		original_offset[Dimension-1] = 1;
		for (SizeType d = Dimension-1; d > 0; --d)
			original_offset[d-1] = size * original_offset[d];
		IndexD offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			offset[d] = original_offset[order[d]];
		}
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), Index<Dimension>::Constant(size), offset);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::Grid(const Grid &o)
		: _data(o._data)
	{
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), o.allSizes());
	}

	////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////
	// DESTRUCTOR
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::~Grid()
	{ }

	////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////////////////////////////
	// DATA
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline const T * Grid<T, Dimension, Allocator>::data() const
	{
		return _data.data();
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline T * Grid<T, Dimension, Allocator>::data()
	{
		return _data.data();
	}

	////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////
	// CONVERSIONS
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline const typename Grid<T, Dimension, Allocator>::Data& Grid<T, Dimension, Allocator>::to_stdvector() const
	{
		return _data;
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline typename Grid<T, Dimension, Allocator>::Data& Grid<T, Dimension, Allocator>::to_stdvector()
	{
		return _data;
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::operator const Data&() const
	{
		return _data;
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator>::operator Data&()
	{
		return _data;
	}

	////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////
	// INFORMATION
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline bool Grid<T, Dimension, Allocator>::empty() const
	{
		return _data.empty();
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline bool Grid<T, Dimension, Allocator>::operator==(const Grid &o) const
	{
		if (&o == this)
			return true;
		for (SizeType i = static_cast<SizeType>(0); i < Dimension; ++i)
			if (DopeVector<T, Dimension>::sizeAt(i) != o.sizeAt(i))
				return false;
		return _data == o._data;
	}

	////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////
	// RESET
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::clear()
	{
		_data.clear();
		DopeVector<T, Dimension>::reset(nullptr, static_cast<SizeType>(0), IndexD::Zero());
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::reset(const T &default_value)
	{
		IndexD size = DopeVector<T, Dimension>::allSizes();
		_data.assign(size.prod(), default_value);
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::resize(const IndexD &size, const T &default_value)
	{
		_data.resize(size.prod(), default_value);
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::resize(const IndexD &size, const IndexD &order, const T &default_value)
	{
		_data.resize(size.prod(), default_value);
		IndexD original_offset;
		original_offset[Dimension-1] = 1;
		for (SizeType d = Dimension-1; d > static_cast<SizeType>(0); --d)
			original_offset[d-1] = size[d] * original_offset[d];
		IndexD offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			offset[d] = original_offset[order[d]];
		}
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size, offset);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::resize(const SizeType size, const T &default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		resize(newSize, default_value);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::resize(const SizeType size, const IndexD &order, const T & default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		resize(newSize, order, default_value);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::assign(const IndexD &size, const T &default_value)
	{
		_data.assign(size.prod(), default_value);
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::assign(const IndexD &size, const IndexD &order, const T &default_value)
	{
		_data.assign(size.prod(), default_value);
		IndexD original_offset;
		original_offset[Dimension-1] = 1;
		for (SizeType d = Dimension-1; d > static_cast<SizeType>(0); --d)
			original_offset[d-1] = size[d] * original_offset[d];
		IndexD offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			offset[d] = original_offset[order[d]];
		}
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size, offset);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::assign(const SizeType size, const T &default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		assign(newSize, default_value);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::assign(const SizeType size, const IndexD &order, const T &default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		assign(newSize, order, default_value);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::conservativeResize(const IndexD &size, const T &default_value)
	{
		Grid<T, Dimension, Allocator> newGrid(size, default_value);
		IndexD oldSize = DopeVector<T, Dimension>::allSizes();
		IndexD minSize;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			minSize[d] = std::min(oldSize[d], size[d]);
		IndexD start = IndexD::Zero();
		DopeVector<T, Dimension> oldWindow = DopeVector<T, Dimension>::window(start, minSize);
		DopeVector<T, Dimension> newWindow = newGrid.window(start, minSize);
		newWindow.import(oldWindow);
		*this = std::move(newGrid);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::conservativeResize(const IndexD &size, const IndexD &order, const T &default_value)
	{
		IndexD original_offset;
		original_offset[Dimension-1] = 1;
		for (SizeType d = Dimension-1; d > static_cast<SizeType>(0); --d)
			original_offset[d-1] = size[d] * original_offset[d];
		IndexD offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			offset[d] = original_offset[order[d]];
		}
		Grid<T, Dimension, Allocator> newGrid(size, offset, default_value);
		IndexD oldSize = DopeVector<T, Dimension>::allSizes();
		IndexD minSize;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			minSize[d] = std::min(oldSize[d], size[d]);
		IndexD start = IndexD::Zero();
		DopeVector<T, Dimension> oldWindow = DopeVector<T, Dimension>::window(start, minSize);
		DopeVector<T, Dimension> newWindow = newGrid.window(start, minSize);
		newWindow.import(oldWindow);
		*this = std::move(newGrid);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::conservativeResize(const SizeType size, const T &default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		conservativeResize(newSize, default_value);
	}

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::conservativeResize(const SizeType size, const IndexD &order, const T &default_value)
	{
		IndexD newSize = IndexD::Constant(size);
		conservativeResize(newSize, order, default_value);
	}

	////////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////////
	// ASSIGNMENTS
	////////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension, class Allocator >
	inline Grid<T, Dimension, Allocator> & Grid<T, Dimension, Allocator>::operator=(const Grid &o)
	{
		if (&o != this) {
			_data = o._data;
			DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), o.allSizes());
		}
		return *this;
	}

#ifdef DOPE_USE_RTTI
	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::import(const DopeVector<T, Dimension> &o)
	{
		if (&o == this)
			return;

		try {
			const Grid<T, Dimension, Allocator> &oo = dynamic_cast<const Grid<T, Dimension, Allocator> &>(o);
			IndexD size = DopeVector<T, Dimension>::allSizes();
			for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
				if (size[d] != oo.sizeAt(d))
					throw std::out_of_range("Matrixes do not have same size.");
			_data = oo._data;
			DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size); // could be unuseful
		} catch(std::bad_cast &bc) {
			DopeVector<T, Dimension>::import(o);
		}
	}
#endif

	template < typename T, SizeType Dimension, class Allocator >
	inline void Grid<T, Dimension, Allocator>::swap(Grid &o)
	{
		_data.swap(o._data);
		IndexD size = DopeVector<T, Dimension>::allSizes();
		DopeVector<T, Dimension>::reset(_data.data(), static_cast<SizeType>(0), size);
	}

	////////////////////////////////////////////////////////////////////////////
	
}
