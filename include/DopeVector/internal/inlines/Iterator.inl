// Copyright (c) 2016 Giorgio Marcias & Maurizio Kovacic
//
// This source code is part of DopeVector header library
// and it is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com
// Author: Maurizio Kovacic
// email: maurizio.kovacic@gmail.com

#include <DopeVector/internal/Iterator.hpp>
#include <limits>

namespace dope {

	namespace internal {

		////////////////////////////////////////////////////////////////////////
		// CONSTRUCTORS
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline Iterator<T, Dimension, Const>::Iterator()
		    : _currentIndex(IndexD::Zero())
		    , _valid(false)
		{ }

		template < typename T, SizeType Dimension, bool Const >
		inline Iterator<T, Dimension, Const>::Iterator(DopeVectorType &dope_vector, const SizeType i, const bool valid)
		    : _data(dope_vector)
		    , _currentIndex(valid ? dope::to_index<Dimension>(i, dope_vector.allSizes()) : IndexD::Zero())
		    , _valid(valid)
		{
			if (_valid && i >= _data.get().size()) {
				_currentIndex = IndexD::Zero();
				_valid = false;
			}
		}

		template < typename T, SizeType Dimension, bool Const >
		inline Iterator<T, Dimension, Const>::Iterator(DopeVectorType &dope_vector, const IndexD &index, const bool valid)
		    : _data(dope_vector)
		    , _currentIndex(valid ? index : IndexD::Zero())
		    , _valid(valid)
		{
			if (_valid) {
				IndexD size = _data.get().allSizes();
				for (SizeType d = 0; d < Dimension; ++d)
					if (_currentIndex[d] >= size[d]) {
						_currentIndex = IndexD::Zero();
						_valid = false;
						break;
					}
			}
		}

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// INFORMATION
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline SizeType Iterator<T, Dimension, Const>::to_original() const
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			return _data.get().accumulatedOffset(_currentIndex);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline SizeType Iterator<T, Dimension, Const>::to_position() const
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			return dope::to_position< Dimension >(_currentIndex, _data.get().allSizes());
		}

		template < typename T, SizeType Dimension, bool Const >
		inline const typename Iterator<T, Dimension, Const>::IndexD & Iterator<T, Dimension, Const>::to_index() const
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			return _currentIndex;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::valid() const
		{
			return _valid;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline Iterator<T, Dimension, Const>::operator bool() const
		{
			return valid();
		}

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// DATA ACCESS METHODS
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::reference Iterator<T, Dimension, Const>::operator*() const
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			return const_cast<reference>(_data.get().at(_currentIndex));
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::pointer Iterator<T, Dimension, Const>::operator->() const
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			return const_cast<pointer>(&_data.get().at(_currentIndex));
		}
		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::reference Iterator<T, Dimension, Const>::operator[](const SizeType n) const
		{
			return *(*this + n);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::reference Iterator<T, Dimension, Const>::operator[](const IndexD &n) const
		{
			return *(*this + n);
		}

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// INCREMENT OPERATIONS
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type& Iterator<T, Dimension, Const>::operator++()
		{
			return *this += static_cast<SizeType>(1);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator++(int)
		{
			self_type copy(*this);
			++*this;
			return copy;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type & Iterator<T, Dimension, Const>::operator+=(const SizeType n)
		{
			if (!_valid)
				return *this;
			Index<Dimension> size = _data.get().allSizes();
			SizeType tmp_val, carry = n;
			for (SizeType D = Dimension; D > 0 && carry != static_cast<SizeType>(0); --D) {
				SizeType d = D - static_cast<SizeType>(1);
				tmp_val = _currentIndex[d] + carry;
				carry = tmp_val / size[d];
				_currentIndex[d] = tmp_val % size[d];
			}
			if (carry != static_cast<SizeType>(0)) {
				_currentIndex = IndexD::Zero();
				_valid = false;
			}
			return *this;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator+ (const SizeType n) const {
			self_type copy(*this);
			copy += n;
			return copy;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type & Iterator<T, Dimension, Const>::operator+=(const Index<Dimension> &n)
		{
			if (!_valid)
				return *this;
			Index<Dimension> size = _data.get().allSizes();
			SizeType tmp_val, carry = static_cast<SizeType>(0);
			for (SizeType D = Dimension; D > static_cast<SizeType>(0); --D) {
				SizeType d = D - static_cast<SizeType>(1);
				tmp_val = _currentIndex[d] + n[d] + carry;
				carry = tmp_val / size[d];
				_currentIndex[d] = tmp_val % size[d];
			}
			if (carry != static_cast<SizeType>(0)) {
				_currentIndex = IndexD::Zero();
				_valid = false;
			}
			return *this;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator+ (const Index<Dimension> &n) const {
			self_type copy(*this);
			copy += n;
			return copy;
		}

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// DECREMENT OPERATIONS
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type & Iterator<T, Dimension, Const>::operator--() {
			return *this -= static_cast<SizeType>(1);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator--(int) {
			self_type copy(*this);
			--*this;
			return copy;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type & Iterator<T, Dimension, Const>::operator-=(const SizeType n)
		{
			if (!_valid)
				return *this;
			Index<Dimension> size = _data.get().allSizes();
			difference_type tmp_val, loan, carry = -static_cast<difference_type>(n);
			for (SizeType D = Dimension; D > static_cast<SizeType>(0) && carry != static_cast<difference_type>(0); --D) {
				SizeType d = D - static_cast<SizeType>(1);
				tmp_val = static_cast<difference_type>(_currentIndex[d]) + carry;
				loan = static_cast<difference_type>(0);
				if (tmp_val < static_cast<difference_type>(0))
					loan = std::abs(tmp_val) / static_cast<difference_type>(size[d]) + (std::abs(tmp_val) % static_cast<difference_type>(size[d]) != static_cast<SizeType>(0) ? static_cast<difference_type>(1) : static_cast<difference_type>(0));
				tmp_val += loan * static_cast<difference_type>(size[d]);
				carry = tmp_val / static_cast<difference_type>(size[d]) - loan;
				_currentIndex[d] = static_cast<SizeType>(tmp_val) % size[d];
			}
			if (carry != static_cast<difference_type>(0)) {
				_currentIndex = IndexD::Zero();
				_valid = false;
			}
			return *this;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator- (const SizeType n) const
		{
			self_type copy(*this);
			copy -= n;
			return copy;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type & Iterator<T, Dimension, Const>::operator-=(const Index<Dimension> &n)
		{
			if (!_valid)
				throw std::range_error("Iterator not valid.");
			Index<Dimension> size = _data.get().allSizes();
			difference_type tmp_val, loan, carry = static_cast<difference_type>(0);
			for (SizeType D = Dimension; D > static_cast<SizeType>(0); --D) {
				SizeType d = D - static_cast<SizeType>(1);
				tmp_val = static_cast<difference_type>(_currentIndex[d]) + carry - static_cast<difference_type>(n[d]);
				loan = static_cast<difference_type>(0);
				if (tmp_val < static_cast<difference_type>(0))
					loan = std::abs(tmp_val) / static_cast<difference_type>(size[d]) + (std::abs(tmp_val) % static_cast<difference_type>(size[d]) != static_cast<SizeType>(0) ? static_cast<difference_type>(1) : static_cast<difference_type>(0));
				tmp_val += loan * static_cast<difference_type>(size[d]);
				carry = tmp_val / static_cast<difference_type>(size[d]) - loan;
				_currentIndex[d] = static_cast<SizeType>(tmp_val) % size[d];
			}
			if (carry != static_cast<difference_type>(0)) {
				_currentIndex = IndexD::Zero();
				_valid = false;
			}
			return *this;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::self_type Iterator<T, Dimension, Const>::operator-(const Index<Dimension> &n) const
		{
			self_type copy(*this);
			copy -= n;
			return copy;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline typename Iterator<T, Dimension, Const>::difference_type Iterator<T, Dimension, Const>::operator- (const self_type &o) const
		{
			if (!_valid || !o._valid)
				throw std::range_error("Iterator not valid.");
			if(&_data.get() == &o._data.get())
				return static_cast<difference_type>(o.to_position()) - static_cast<difference_type>(to_position());
			return std::numeric_limits<difference_type>::max();
		}

		////////////////////////////////////////////////////////////////////////



		////////////////////////////////////////////////////////////////////////
		// BOOLEAN OPERATIONS
		////////////////////////////////////////////////////////////////////////

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator==(const self_type &o) const
		{
			return &_data.get() == &o._data.get() && ((!_valid && !o._valid) || (_valid && o._valid && _currentIndex.isApprox(o._currentIndex)));
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator!=(const self_type &o) const
		{
			return !(*this == o);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator< (const self_type &o) const
		{
			if (&_data.get() != &o._data.get())
				throw std::logic_error("Iterators on different dope vectors is undefined.");
			return (_valid && !o._valid) || (_valid && o._valid && to_position() < o.to_position());
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator<=(const self_type &o) const
		{
			return !(o < *this);
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator> (const self_type &o) const
		{
			return o < *this;
		}

		template < typename T, SizeType Dimension, bool Const >
		inline bool Iterator<T, Dimension, Const>::operator>=(const self_type &o) const
		{
			return !(*this < o);
		}

		////////////////////////////////////////////////////////////////////////

	} // namespace internal

} // namespace dope
