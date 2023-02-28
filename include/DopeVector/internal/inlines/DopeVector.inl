// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#include <memory>
#include <array>
#include <DopeVector/DopeVector.hpp>

namespace dope {

	////////////////////////////////////////////////////////////////////////
	// CONSTRUCTORS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension>::DopeVector()
	    : _array(nullptr)
	    , _accumulatedOffset(static_cast<SizeType>(0))
	    , _size(IndexD::Zero())
	    , _offset(IndexD::Zero())
	{ }

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension>::DopeVector(T *array, const SizeType accumulatedOffset, const IndexD &size)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? size : static_cast<const IndexD &>(IndexD::Zero()))
	    , _offset(IndexD::Zero())
	{
		if (array) {
			_offset[Dimension-1] = 1;
			for (SizeType j = Dimension-1; j > static_cast<SizeType>(0); --j)
				_offset[j-1] = _size[j] * _offset[j];
		}
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension>::DopeVector(T *array, const SizeType accumulatedOffset, const IndexD &size, const IndexD &offset)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? size : static_cast<const IndexD &>(IndexD::Zero()))
	    , _offset(array ? offset : static_cast<const IndexD &>(IndexD::Zero()))
	{ }

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ASSIGNMENT OPERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::reset(T *array, const SizeType accumulatedOffset, const IndexD &size)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? size : static_cast<const IndexD &>(IndexD::Zero());
		_offset = IndexD::Zero();
		if (_array) {
			_offset[Dimension-1] = 1;
			for (SizeType j = Dimension-1; j > static_cast<SizeType>(0); --j)
				_offset[j-1] = _size[j] * _offset[j];
		}
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::reset(T *array, const SizeType accumulatedOffset, const IndexD &size, const IndexD &offset)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? size : static_cast<const IndexD &>(IndexD::Zero());
		_offset = _array ? offset : static_cast<const IndexD &>(IndexD::Zero());
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::import(const DopeVector<T, Dimension> &o)
	{
		if (&o == this)
			return;
		if (_size[0] != o._size[0])
			throw std::out_of_range("Matrixes do not have same size.");
		for (SizeType i = static_cast<SizeType>(0); i < _size[0]; ++i)
			operator[](i).import(o[i]);
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::safeImport(const DopeVector<T, Dimension> &o)
	{
		if (&o == this)
			return;
		if (allSizes() != o.allSizes())
			throw std::out_of_range("Matrixes do not have same size.");
		std::unique_ptr<T[]> tempPtr(new T[o.size()]);
		DopeVector<T, Dimension> tmpDopeVector(tempPtr.get(), static_cast<SizeType>(0), o.allSizes());
		tmpDopeVector.import(o);
		import(tmpDopeVector);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ACCESS METHODS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::at(const SizeType i, DopeVector<T, Dimension-1> &s) const
	{
		if (i >= _size[0]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		Index<Dimension-1> new_size, new_offset;
		for (SizeType j = static_cast<SizeType>(1); j < Dimension; ++j) {
			new_size[j-1] = _size[j];
			new_offset[j-1] = _offset[j];
		}
		s.reset(_array + _offset[0] * i, accumulatedOffset(i), new_size, new_offset);
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension-1> DopeVector<T, Dimension>::at(const SizeType i) const
	{
		DopeVector<T, Dimension-1> s;
		at(i, s);
		return s;
	}

	template < typename T, SizeType Dimension >
	inline const T & DopeVector<T, Dimension>::at(const IndexD &i) const
	{
		SizeType offset = static_cast<SizeType>(0);
		for(SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			offset += i[d] * _offset[d];
		return *(_array + offset);
	}

	template < typename T, SizeType Dimension >
	inline T & DopeVector<T, Dimension>::at(const IndexD &i)
	{
		SizeType offset = static_cast<SizeType>(0);
		for(SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			offset += i[d] * _offset[d];
		return *(_array + offset);
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension-1> DopeVector<T, Dimension>::operator[](const SizeType i) const
	{
		DopeVector<T, Dimension-1> s;
		at(i, s);
		return s;
	}

	template < typename T, SizeType Dimension >
	inline const T & DopeVector<T, Dimension>::operator[](const IndexD &i) const
	{
		return at(i);
	}

	template < typename T, SizeType Dimension >
	inline T & DopeVector<T, Dimension>::operator[](const IndexD &i)
	{
		return at(i);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ITERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::iterator DopeVector<T, Dimension>::begin()
	{
		return iterator(*this, static_cast<SizeType>(0), true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::iterator DopeVector<T, Dimension>::end()
	{
		return iterator(*this, static_cast<SizeType>(0), false);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::begin() const
	{
		return cbegin();
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::end() const
	{
		return cend();
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::cbegin() const
	{
		return const_iterator(*this, static_cast<SizeType>(0), true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::cend() const
	{
		return const_iterator(*this, static_cast<SizeType>(0), false);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::iterator DopeVector<T, Dimension>::to_iterator(const SizeType i)
	{
		IndexD orig = dope::to_index(iterator(*this, i, true).to_original(), accumulatedOffset(IndexD::Zero()), _offset);
		return iterator(*this, i, true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::iterator DopeVector<T, Dimension>::to_iterator_from_original(const SizeType i)
	{
		return iterator(*this, dope::to_index(i, _accumulatedOffset, _offset), true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::iterator DopeVector<T, Dimension>::to_iterator(const IndexD &i)
	{
		return iterator(*this, i, true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::to_const_iterator(const SizeType i) const
	{
		return const_iterator(*this, i, true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::to_const_iterator_from_original(const SizeType i) const
	{
		return const_iterator(*this, dope::to_index(i, _accumulatedOffset, _offset), true);
	}

	template < typename T, SizeType Dimension >
	inline typename DopeVector<T, Dimension>::const_iterator DopeVector<T, Dimension>::to_const_iterator(const IndexD &i) const
	{
		return const_iterator(*this, i, true);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// REDUCTION METHODS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::slice(const SizeType d, const SizeType i, DopeVector<T, Dimension-1> &s) const
	{
		if (d >= Dimension) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << Dimension-1 << ']';
			throw std::out_of_range(stream.str());
		}
		if (i >= _size[d]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[d]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		Index<Dimension-1> new_size, new_offset;
		SizeType k = 0;
		for (SizeType j = static_cast<SizeType>(0); j < d; ++j, ++k) {
			new_size[k] = _size[j];
			new_offset[k] = _offset[j];
		}
		for (SizeType j = d+1; j < Dimension; ++j, ++k) {
			new_size[k] = _size[j];
			new_offset[k] = _offset[j];
		}
		s.reset(_array + _offset[d] * i, accumulatedOffset(i, d), new_size, new_offset);
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension-1> DopeVector<T, Dimension>::slice(const SizeType d, const SizeType i) const
	{
		DopeVector<T, Dimension-1> s;
		slice(d, i, s);
		return s;
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::permute(const IndexD &order, DopeVector &p) const
	{
		std::array<bool, Dimension> included = {{false}};
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (order[d] >= Dimension) {
				std::stringstream stream;
				stream << "Index " << order[d] << " is out of range [0, " << Dimension-1 << ']';
				throw std::out_of_range(stream.str());
			}
			if (included[order[d]]) {
				std::stringstream stream;
				stream << "Dimension " << order[d] << " duplicated.";
				throw std::invalid_argument(stream.str());
			}
			included[order[d]] = true;
		}
		IndexD new_size, new_offset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			new_size[d] = _size[order[d]];
			new_offset[d] = _offset[order[d]];
		}
		p.reset(_array, _accumulatedOffset, new_size, new_offset);
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension> DopeVector<T, Dimension>::permute(const IndexD &order) const
	{
		DopeVector<T, Dimension> p;
		permute(order, p);
		return p;
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::window(const IndexD &start, const IndexD &size, DopeVector<T, Dimension> &w) const
	{
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (start[d] >= _size[d]) {
				std::stringstream stream;
				stream << "Index " << start[d] << " is out of range [0, " << _size[d] << ']';
				throw std::out_of_range(stream.str());
			}
			if (start[d] + size[d] > _size[d]) {
				std::stringstream stream;
				stream << "Window size " << size[d] << " is out of range [" << 0 << ", " << _size[d] - start[d] << ']';
				throw std::out_of_range(stream.str());
			}
		}
		SizeType newAccumulatedOffset = _accumulatedOffset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d)
			newAccumulatedOffset += _offset[d] * start[d];
		w.reset(_array + (newAccumulatedOffset - _accumulatedOffset), newAccumulatedOffset, size, _offset);
	}

	template < typename T, SizeType Dimension >
	inline DopeVector<T, Dimension> DopeVector<T, Dimension>::window(const IndexD &start, const IndexD &size) const
	{
		DopeVector<T, Dimension> w;
		window(start, size, w);
		return w;
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// INFORMATION
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline SizeType DopeVector<T, Dimension>::sizeAt(const SizeType d) const
	{
		if (d >= Dimension) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << Dimension-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return _size[d];
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::allSizes(typename DopeVector<T, Dimension>::IndexD &s) const
	{
		s = _size;
	}

	template < typename T, SizeType Dimension >
	inline const typename DopeVector<T, Dimension>::IndexD & DopeVector<T, Dimension>::allSizes() const
	{
		return _size;
	}

	template < typename T, SizeType Dimension >
	inline SizeType DopeVector<T, Dimension>::size() const
	{
		SizeType total = _size[0];
		for (SizeType i = static_cast<SizeType>(1); i < Dimension; ++i)
			total *= _size[i];
		return total;
	}

	template < typename T, SizeType Dimension >
	inline SizeType DopeVector<T, Dimension>::offsetAt(const SizeType d) const
	{
		if (d >= Dimension) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << Dimension-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return _offset[d];
	}

	template < typename T, SizeType Dimension >
	inline void DopeVector<T, Dimension>::allOffsets(typename DopeVector<T, Dimension>::IndexD &o) const
	{
		o = _offset;
	}

	template < typename T, SizeType Dimension >
	inline const typename DopeVector<T, Dimension>::IndexD & DopeVector<T, Dimension>::allOffsets() const
	{
		return _offset;
	}

	template < typename T, SizeType Dimension >
	inline SizeType DopeVector<T, Dimension>::accumulatedOffset(const SizeType i, const SizeType d) const
	{
		if (d >= Dimension) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << Dimension-1 << ']';
			throw std::out_of_range(stream.str());
		}
		if (i >= _size[d]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[d]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return  _accumulatedOffset + _offset[d] * i;
	}

	template < typename T, SizeType Dimension >
	inline SizeType DopeVector<T, Dimension>::accumulatedOffset(const IndexD &index) const
	{
		SizeType accumulatedOffset = _accumulatedOffset;
		for (SizeType d = static_cast<SizeType>(0); d < Dimension; ++d) {
			if (d >= _size[d]) {
				std::stringstream stream;
				stream << "Index " << d << " is out of range [0, " << _size[d]-1 << ']';
				throw std::out_of_range(stream.str());
			}
			accumulatedOffset += index[d] * _offset[d];
		}
		return accumulatedOffset;
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// RELATIONAL OPERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T, SizeType Dimension >
	inline bool DopeVector<T, Dimension>::operator==(const DopeVector<T, Dimension> &r) const
	{
		if (_array == r._array && _size == r._size && _offset == r._offset)
			return true;
		if (_size != r._size)
			return false;
		for (SizeType i = static_cast<SizeType>(0); i < _size[0]; ++i)
			if (at(i) != r.at(i))
				return false;
		return true;
	}

	template < typename T, SizeType Dimension >
	inline bool DopeVector<T, Dimension>::operator!=(const DopeVector<T, Dimension> &r) const
	{
		return !(*this == r);
	}

    ////////////////////////////////////////////////////////////////////////





	////////////////////////////////////////////////////////////////////////
	// CONSTRUCTORS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline DopeVector<T, 1>::DopeVector()
	    : _array(nullptr)
	    , _accumulatedOffset(static_cast<SizeType>(0))
	    , _size(Index1::Zero())
	    , _offset(Index1::Zero())
	{ }

	template < typename T >
	inline DopeVector<T, 1>::DopeVector(T *array, const SizeType accumulatedOffset, const SizeType size)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? Index1::Constant(size) : Index1::Zero())
	    , _offset(array ? Index1::Ones() : Index1::Zero())
	{ }

	template < typename T >
	inline DopeVector<T, 1>::DopeVector(T *array, const SizeType accumulatedOffset, const SizeType size, const SizeType offset)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? Index1::Constant(size) : Index1::Zero())
	    , _offset(array ? Index1::Constant(offset) : Index1::Zero())
	{ }

	template < typename T >
	inline DopeVector<T, 1>::DopeVector(T *array, const SizeType accumulatedOffset, const Index1 &size)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? size : static_cast<const Index1 &>(Index1::Zero()))
	    , _offset(array ? Index1::Ones() : Index1::Zero())
	{ }

	template < typename T >
	inline DopeVector<T, 1>::DopeVector(T *array, const SizeType accumulatedOffset, const Index1 &size, const Index1 &offset)
	    : _array(array)
	    , _accumulatedOffset(array ? accumulatedOffset : static_cast<SizeType>(0))
	    , _size(array ? size : static_cast<const Index1 &>(Index1::Zero()))
	    , _offset(array ? offset : static_cast<const Index1 &>(Index1::Zero()))
	{ }

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ASSIGNMENT OPERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline void DopeVector<T, 1>::reset(T *array, const SizeType accumulatedOffset, const SizeType size)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? Index1::Constant(size) : Index1::Zero();
		_offset = _array ? Index1::Ones() : Index1::Zero();
	}

	template < typename T >
	inline void DopeVector<T, 1>::reset(T *array, const SizeType accumulatedOffset, const SizeType size, const SizeType offset)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? Index1::Constant(size) : Index1::Zero();
		_offset = _array ? Index1::Constant(offset) : Index1::Zero();
	}

	template < typename T >
	inline void DopeVector<T, 1>::reset(T *array, const SizeType accumulatedOffset, const Index1 &size)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? size : static_cast<const Index1 &>(Index1::Zero());
		_offset = _array ? Index1::Ones() : Index1::Zero();
	}

	template < typename T >
	inline void DopeVector<T, 1>::reset(T *array, const SizeType accumulatedOffset, const Index1 &size, const Index1 &offset)
	{
		_array = array;
		_accumulatedOffset = _array ? accumulatedOffset : static_cast<SizeType>(0);
		_size = _array ? size : static_cast<const Index1 &>(Index1::Zero());
		_offset = _array ? offset : static_cast<const Index1 &>(Index1::Zero());
	}

	template < typename T >
	inline void DopeVector<T, 1>::import(const DopeVector<T, 1> &o)
	{
		if (&o == this)
			return;
		if (_size[0] != o._size[0])
			throw std::out_of_range("Matrixes do not have same size.");
		for (SizeType i = static_cast<SizeType>(0); i < _size[0]; ++i)
			at(i) = o.at(i);
	}

	template < typename T >
	inline void DopeVector<T, 1>::safeImport(const DopeVector<T, 1> &o)
	{
		if (&o == this)
			return;
		if (sizeAt(0) != o.sizeAt(0))
			throw std::out_of_range("Matrixes do not have same size.");
		std::unique_ptr<T[]> tempPtr(new T[o.size()]);
		DopeVector<T, 1> tmpDopeVector(tempPtr.get(), static_cast<SizeType>(0), o.allSizes());
		tmpDopeVector.import(o);
		import(tmpDopeVector);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ACCESS METHODS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline const T & DopeVector<T, 1>::at(const SizeType i) const
	{
		if (i >= _size[0]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return *(_array + i * _offset[0]);
	}

	template < typename T >
	inline T & DopeVector<T, 1>::at(const SizeType i)
	{
		if (i >= _size[0]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return *(_array + i * _offset[0]);
	}

	template < typename T >
	inline const T & DopeVector<T, 1>::at(const Index1 i) const
	{
		return at(i[0]);
	}

	template < typename T >
	inline T & DopeVector<T, 1>::at(const Index1 i)
	{
		return at(i[0]);
	}

	template < typename T >
	inline const T & DopeVector<T, 1>::operator[](const SizeType i) const
	{
		return at(i);
	}

	template < typename T >
	inline T & DopeVector<T, 1>::operator[](const SizeType i)
	{
		return at(i);
	}

	template < typename T >
	inline const T & DopeVector<T, 1>::operator[](const Index1 i) const
	{
		return at(i[0]);
	}

	template < typename T >
	inline T & DopeVector<T, 1>::operator[](const Index1 i)
	{
		return at(i[0]);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// ITERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline typename DopeVector<T, 1>::iterator DopeVector<T, 1>::begin()
	{
		return iterator(*this, static_cast<SizeType>(0), true);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::iterator DopeVector<T, 1>::end()
	{
		return iterator(*this, static_cast<SizeType>(0), false);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::begin() const
	{
		return cbegin();
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::end() const
	{
		return cend();
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::cbegin() const
	{
		return const_iterator(*this, static_cast<SizeType>(0), true);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::cend() const
	{
		return const_iterator(*this, static_cast<SizeType>(0), false);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::iterator DopeVector<T, 1>::to_iterator(const SizeType i)
	{
		return iterator(*this, i, true);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::iterator DopeVector<T, 1>::to_iterator(const Index1 &i)
	{
		return iterator(*this, i, true);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::to_const_iterator(const SizeType i) const
	{
		return const_iterator(*this, i, true);
	}

	template < typename T >
	inline typename DopeVector<T, 1>::const_iterator DopeVector<T, 1>::to_const_iterator(const Index1 &i) const
	{
		return const_iterator(*this, i, true);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// REDUCTION METHODS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline const T & DopeVector<T, 1>::slice(const SizeType i) const
	{
		return at(i);
	}

	template < typename T >
	inline T & DopeVector<T, 1>::slice(const SizeType i)
	{
		return at(i);
	}

	template < typename T >
	inline void DopeVector<T, 1>::permute(const Index1 &order, DopeVector<T, 1> &p) const
	{
		if (order[0] >= 1) {
			std::stringstream stream;
			stream << "Index " << order[0] << " is out of range [0, 0]";
			throw std::out_of_range(stream.str());
		}
		if (&p == this)
			return;
		Index1 new_size, new_offset;
		new_size[0] = _size[order[0]];
		new_offset[0] = _offset[order[0]];
		p.reset(_array, _accumulatedOffset, new_size, new_offset);
	}

	template < typename T >
	inline DopeVector<T, 1> DopeVector<T, 1>::permute(const Index1 &order) const
	{
		DopeVector<T, 1> p;
		permute(order, p);
		return p;
	}

	template < typename T >
	inline void DopeVector<T, 1>::window(const Index1 &start, const Index1 &size, DopeVector<T, 1> &w) const
	{
		if (start[0] >= _size[0]) {
			std::stringstream stream;
			stream << "Index " << start[0] << " is out of range [0, " << _size[0] << ']';
			throw std::out_of_range(stream.str());
		}
		if (start[0] + size[0] > _size[0]) {
			std::stringstream stream;
			stream << "Window size " << size[0] << " is out of range [" << 0 << ", " << _size[0] - start[0] << ']';
			throw std::out_of_range(stream.str());
		}
		SizeType accumulatedOffset = _accumulatedOffset + _offset[0] * start[0];
		w.reset(_array + (accumulatedOffset - _accumulatedOffset), accumulatedOffset, size, _offset);
	}

	template < typename T >
	inline DopeVector<T, 1> DopeVector<T, 1>::window(const Index1 &start, const Index1 &size) const
	{
		DopeVector<T, 1> w;
		window(start, size, w);
		return w;
	}

	template < typename T >
	inline void DopeVector<T, 1>::window(const SizeType start, const SizeType size, DopeVector<T, 1> &w) const
	{
		window(Index1::Constant(start), Index1::Constant(size), w);
	}

	template < typename T >
	inline DopeVector<T, 1> DopeVector<T, 1>::window(const SizeType start, const SizeType size) const
	{
		DopeVector<T, 1> w;
		window(start, size, w);
		return w;
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// INFORMATION
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline SizeType DopeVector<T, 1>::sizeAt(const SizeType d) const
	{
		if (d >= static_cast<SizeType>(1)) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << static_cast<SizeType>(0) << ']';
			throw std::out_of_range(stream.str());
		}
		return _size[0];
	}

	template < typename T >
	inline void DopeVector<T, 1>::allSizes(Index1 &s) const
	{
		s = _size;
	}

	template < typename T >
	inline const Index1 & DopeVector<T, 1>::allSizes() const
	{
		return _size;
	}

	template < typename T >
	inline SizeType DopeVector<T, 1>::size() const
	{
		return _size[0];
	}

	template < typename T >
	inline SizeType DopeVector<T, 1>::offsetAt(const SizeType d) const
	{
		if (d >= static_cast<SizeType>(1)) {
			std::stringstream stream;
			stream << "Index " << d << " is out of range [0, " << static_cast<SizeType>(0) << ']';
			throw std::out_of_range(stream.str());
		}
		return _size[0];
	}

	template < typename T >
	inline void DopeVector<T, 1>::allOffsets(Index1 &o) const
	{
		o = _offset;
	}

	template < typename T >
	inline const Index1 & DopeVector<T, 1>::allOffsets() const
	{
		return _offset;
	}

	template < typename T >
	inline SizeType DopeVector<T, 1>::accumulatedOffset(const SizeType i) const
	{
		if (i >= _size[0]) {
			std::stringstream stream;
			stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
			throw std::out_of_range(stream.str());
		}
		return _accumulatedOffset + _offset[0] * i;
	}

	template < typename T >
	inline SizeType DopeVector<T, 1>::accumulatedOffset(const Index1 &index) const
	{
		return accumulatedOffset(index[0]);
	}

	////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	// RELATIONAL OPERATORS
	////////////////////////////////////////////////////////////////////////

	template < typename T >
	inline bool DopeVector<T, 1>::operator==(const DopeVector<T, 1> &r) const
	{
		if (_array == r._array && _size == r._size && _offset == r._offset)
			return true;
		if (_size != r._size)
			return false;
		for (SizeType i = static_cast<SizeType>(0); i < _size[0]; ++i)
			if (at(i) != r.at(i))
				return false;
		return true;
	}

	template < typename T >
	inline bool DopeVector<T, 1>::operator!=(const DopeVector<T, 1> &r) const
	{
		return !(*this == r);
	}

}
