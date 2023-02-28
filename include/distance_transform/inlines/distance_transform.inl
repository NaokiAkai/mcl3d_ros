// Copyright (c) 2016 Giorgio Marcias
//
// This file is part of distance_transform, a C++11 implementation of the
// algorithm in "Distance Transforms of Sampled Functions"
// Pedro F. Felzenszwalb, Daniel P. Huttenlocher
// Theory of Computing, Vol. 8, No. 19, September 2012
//
// This source code is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com

#include <cmath>
#include <distance_transform/distance_transform.hpp>

namespace dt {

	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceTransformL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const bool squared, const std::size_t nThreads)
	{
		dope::Index<DIM> fSize, DSize;
		fSize = f.allSizes();
		DSize = D.allSizes();
		if (DSize != fSize)
			throw std::out_of_range("Matrixes do not have same size.");

		dope::Grid<Scalar, DIM> fCopy(fSize);
		fCopy.import(f);
		dope::Grid<Scalar, DIM> DCopy(DSize);

		dope::DopeVector<Scalar, DIM> tmpF(fCopy);
		dope::DopeVector<Scalar, DIM> tmpD(DCopy);

		dope::Index<DIM> order;

		for (dope::SizeType d = static_cast<dope::SizeType>(0); d < DIM; ++d) {
			// permute rotate
			for (dope::SizeType o = static_cast<dope::SizeType>(0); o < DIM; ++o)
				order[o] = (d + o) % DIM;
			dope::DopeVector<Scalar, DIM> tmpF_rotated = tmpF.permute(order);
			dope::DopeVector<Scalar, DIM> tmpD_rotated = tmpD.permute(order);


			dope::Index<DIM> winStart = dope::Index<DIM>::Zero(), winSize;
			tmpF_rotated.allSizes(winSize);

			dope::SizeType range = winSize[0];
			if (nThreads < range) {
				range += range % nThreads;
				range /= nThreads;
			}
			std::size_t nWindows = winSize[0] / range + (winSize[0] % range != 0 ? 1 : 0);

			if (nWindows > 1) {
				std::vector<dope::DopeVector<Scalar, DIM>> tmpWindowsF(nWindows);
				std::vector<dope::DopeVector<Scalar, DIM>> tmpWindowsD(nWindows);
				std::vector<std::thread> threads(nWindows);

				for (std::size_t i = 0; i < nWindows; ++i) {
					winStart[0] = i * range;
					winSize[0] = std::min(range, tmpF_rotated.sizeAt(0) - winStart[0]);
					tmpWindowsF.at(i) = tmpF_rotated.window(winStart, winSize);
					tmpWindowsD.at(i) = tmpD_rotated.window(winStart, winSize);
					winStart[0] = 0;
					winSize[0] = tmpF_rotated.sizeAt(0);
					threads.at(i) = std::thread(static_cast<void(*)(const dope::DopeVector<Scalar, DIM> &, dope::DopeVector<Scalar, DIM> &)>(&distanceL2Helper), std::cref(tmpWindowsF.at(i)), std::ref(tmpWindowsD.at(i)));
				}
				for (std::size_t i = 0; i < nWindows; ++i)
					threads.at(i).join();
			} else {
				distanceL2Helper(tmpF_rotated, tmpD_rotated);
			}

			std::swap(tmpD, tmpF);
		}

		if (DIM % 2 == 0)
			DCopy = std::move(fCopy);

		D.import(DCopy);

		if (!squared)
			element_wiseSquareRoot(D);
	}



	template < typename Scalar >
	inline void DistanceTransform::distanceTransformL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, const bool squared, const std::size_t)
	{
		dope::Index1 fSize, DSize;
		fSize = f.allSizes();
		DSize = D.allSizes();
		if (DSize != fSize)
			throw std::out_of_range("Matrixes do not have same size.");

		distanceL2(f, D);

		if (!squared)
			element_wiseSquareRoot(D);
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceTransformL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, dope::DopeVector<dope::SizeType, DIM> &I, const bool squared, const std::size_t nThreads)
	{
		dope::Index<DIM> fSize, DSize, ISize;
		fSize = f.allSizes();
		DSize = D.allSizes();
		ISize = I.allSizes();
		if (DSize != fSize || ISize != fSize)
			throw std::out_of_range("Matrixes do not have same size.");

		dope::Grid<Scalar, DIM> fCopy(fSize);
		fCopy.import(f);
		dope::Grid<Scalar, DIM> DCopy(DSize);
		dope::Grid<dope::SizeType, DIM> ICopyPre(ISize), ICopyPost(ISize);
		ICopyPre.import(I);

		dope::DopeVector<Scalar, DIM> tmpF(fCopy);
		dope::DopeVector<Scalar, DIM> tmpD(D);
		dope::DopeVector<dope::SizeType, DIM> Ipre(ICopyPre);
		dope::DopeVector<dope::SizeType, DIM> Ipost(ICopyPost);

		dope::Index<DIM> order;

		for (dope::SizeType d = static_cast<dope::SizeType>(0); d < DIM; ++d) {
			// permute rotate
			for (dope::SizeType o = static_cast<dope::SizeType>(0); o < DIM; ++o)
				order[o] = (d + o) % DIM;
			dope::DopeVector<Scalar, DIM> tmpF_rotated = tmpF.permute(order);
			dope::DopeVector<Scalar, DIM> tmpD_rotated = tmpD.permute(order);
			dope::DopeVector<dope::SizeType, DIM> Ipre_rotated = Ipre.permute(order);
			dope::DopeVector<dope::SizeType, DIM> Ipost_rotated = Ipost.permute(order);

			dope::Index<DIM> winStart = dope::Index<DIM>::Zero(), winSize;
			tmpF_rotated.allSizes(winSize);

			std::size_t range = winSize[0];
			if (nThreads < range) {
				range += range % nThreads;
				range /= nThreads;
			}
			std::size_t nWindows = winSize[0] / range + (winSize[0] % range != 0 ? 1 : 0);

			if (nWindows > 1) {
				std::vector<dope::DopeVector<Scalar, DIM>> tmpWindowsF(nWindows);
				std::vector<dope::DopeVector<Scalar, DIM>> tmpWindowsD(nWindows);
				std::vector<dope::DopeVector<dope::SizeType, DIM>> tmpWindowsIPre(nWindows);
				std::vector<dope::DopeVector<dope::SizeType, DIM>> tmpWindowsIPost(nWindows);
				std::vector<std::thread> threads(nWindows);

				for (std::size_t i = 0; i < nWindows; ++i) {
					winStart[0] = i * range;
					winSize[0] = std::min(range, tmpF_rotated.sizeAt(0) - winStart[0]);
					tmpWindowsF.at(i) = tmpF_rotated.window(winStart, winSize);
					tmpWindowsD.at(i) = tmpD_rotated.window(winStart, winSize);
					tmpWindowsIPre.at(i) = Ipre_rotated.window(winStart, winSize);
					tmpWindowsIPost.at(i) = Ipost_rotated.window(winStart, winSize);
					winStart[0] = 0;
					winSize[0] = tmpF_rotated.sizeAt(0);
					threads.at(i) = std::thread(static_cast<void(*)(const dope::DopeVector<Scalar, DIM> &, dope::DopeVector<Scalar, DIM> &, const dope::DopeVector<dope::SizeType, DIM> &, dope::DopeVector<dope::SizeType, DIM> &)>(&distanceL2Helper), std::cref(tmpWindowsF.at(i)), std::ref(tmpWindowsD.at(i)), std::cref(tmpWindowsIPre.at(i)), std::ref(tmpWindowsIPost.at(i)));
				}
				for (std::size_t i = 0; i < nWindows; ++i)
					threads.at(i).join();
			} else {
				distanceL2Helper(tmpF_rotated, tmpD_rotated, Ipre_rotated, Ipost_rotated);
			}

			std::swap(tmpD, tmpF);
			std::swap(Ipost, Ipre);
		}

		if (DIM % 2 == 0) {
			DCopy = std::move(fCopy);
			ICopyPost = std::move(ICopyPre);
		}

		D.import(DCopy);
		I.import(ICopyPost);

		if (!squared)
			element_wiseSquareRoot(D);
	}




	template < typename Scalar >
	inline void DistanceTransform::distanceTransformL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, dope::DopeVector<dope::SizeType, 1> &I, const bool squared, const std::size_t)
	{
		dope::Index1 fSize, DSize, ISize;
		fSize = f.allSizes();
		DSize = D.allSizes();
		ISize = I.allSizes();
		if (DSize != fSize || ISize != fSize)
			throw std::out_of_range("Matrixes do not have same size.");

		distanceL2(f, D, I);

		if (!squared)
			element_wiseSquareRoot(D);
	}



	template < dope::SizeType DIM >
	inline void DistanceTransform::initializeIndices(dope::DopeVector<dope::SizeType, DIM> &I)
	{
		dope::DopeVector<dope::SizeType, DIM-1> I_q;
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < I.sizeAt(0); ++q) {
			I.at(q, I_q);
			initializeIndices(I_q);
		}
	}



	inline void DistanceTransform::initializeIndices(dope::DopeVector<dope::SizeType, 1> &I)
	{
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < I.sizeAt(0); ++q)
			I[q] = I.accumulatedOffset(q);
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceL2Helper(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D)
	{
		dope::DopeVector<Scalar, DIM-1> f_dq;
		dope::DopeVector<Scalar, DIM-1> D_dq;

		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			f.slice(0, q, f_dq);
			D.slice(0, q, D_dq);
			distanceL2(f_dq, D_dq);
		}
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D)
	{
		dope::DopeVector<Scalar, DIM-1> f_q, D_q;
		// compute distance at lower dimensions for each hyperplane
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			f.at(q, f_q);
			D.at(q, D_q);
			distanceL2(f_q, D_q);
		}
	}



	template < typename Scalar >
	inline void DistanceTransform::distanceL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D)
	{
		if (f.sizeAt(0) == static_cast<dope::SizeType>(0) || f.sizeAt(0) > D.sizeAt(0))
			return;
		if (f.sizeAt(0) == static_cast<dope::SizeType>(1)) {
			D[0] = f[0];
			return;
		}
		dope::SizeType k = static_cast<dope::SizeType>(0);          // index of rightmost parabola in lower envelope
		dope::SizeType *v = new dope::SizeType[f.sizeAt(0)];	    // locations of parabolas in lower envelope
		double *z = new double[f.sizeAt(0) + 1];        // locations of boundaries between parabolas
		double s = double(0);
		// initialization
		v[0] = static_cast<dope::SizeType>(0);
		z[0] = -std::numeric_limits<double>::max();
		z[1] = std::numeric_limits<double>::max();
		// compute lowest envelope:
		for (dope::SizeType q = static_cast<dope::SizeType>(1); q < f.sizeAt(0); ++q) {
			++k;    // this compensates for first line of next do-while block
			do {
				--k;
				// compute horizontal position of intersection between the parabola from q and the current lowest parabola
				s = ((f[q] + q*q) - static_cast<double>(f[v[k]] + v[k]*v[k])) / (2*q - static_cast<double>(2*v[k]));
			} while (s <= z[k]);
			++k;
			v[k] = q;
			z[k] = s;
			z[k+1] = std::numeric_limits<double>::max();
		}
		// fill in values of distance transform
		k = static_cast<dope::SizeType>(0);
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			while(z[k+1] < static_cast<double>(q))
				++k;
			D[q] = f[v[k]] + (q - static_cast<Scalar>(v[k]))*(q - static_cast<Scalar>(v[k]));
		}
		// delete allocated memory
		delete[] z;
		delete[] v;
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceL2Helper(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const dope::DopeVector<dope::SizeType, DIM> &Ipre, dope::DopeVector<dope::SizeType, DIM> &Ipost)
	{
		dope::DopeVector<Scalar, DIM-1> f_dq;
		dope::DopeVector<Scalar, DIM-1> D_dq;
		dope::DopeVector<dope::SizeType, DIM-1> Ipre_dq;
		dope::DopeVector<dope::SizeType, DIM-1> Ipost_dq;

		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			f.slice(0, q, f_dq);
			D.slice(0, q, D_dq);
			Ipre.slice(0, q, Ipre_dq);
			Ipost.slice(0, q, Ipost_dq);
			distanceL2(f_dq, D_dq, Ipre_dq, Ipost_dq);
		}
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::distanceL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const dope::DopeVector<dope::SizeType, DIM> &Ipre, dope::DopeVector<dope::SizeType, DIM> &Ipost)
	{
		dope::DopeVector<Scalar, DIM-1> f_q, D_q;
		dope::DopeVector<dope::SizeType, DIM-1> Ipre_q, Ipost_q;
		// compute distance at lower dimensions for each hyperplane
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			f.at(q, f_q);
			D.at(q, D_q);
			Ipre.at(q, Ipre_q);
			Ipost.at(q, Ipost_q);
			distanceL2(f_q, D_q, Ipre_q, Ipost_q);
		}
	}



	template < typename Scalar >
	inline void DistanceTransform::distanceL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, const dope::DopeVector<dope::SizeType, 1> &Ipre, dope::DopeVector<dope::SizeType, 1> &Ipost)
	{
		if (f.sizeAt(0) == static_cast<dope::SizeType>(0) || f.sizeAt(0) > D.sizeAt(0))
			return;
		if (f.sizeAt(0) == static_cast<dope::SizeType>(1)) {
			D[0] = f[0];
			Ipost[0] = Ipre[0];
			return;
		}
		dope::SizeType k = static_cast<dope::SizeType>(0);	        // index of rightmost parabola in lower envelope
		dope::SizeType *v = new dope::SizeType[f.sizeAt(0)];        // locations of parabolas in lower envelope
		double *z = new double[f.sizeAt(0) + 1];        // locations of boundaries between parabolas
		double s = double(0);
		// initialization
		v[0] = static_cast<dope::SizeType>(0);
		z[0] = -std::numeric_limits<double>::max();
		z[1] = std::numeric_limits<double>::max();
		// compute lowest envelope:
		for (dope::SizeType q = static_cast<dope::SizeType>(1); q < f.sizeAt(0); ++q) {
			++k;    // this compensates for first line of next do-while block
			do {
				--k;
				// compute horizontal position of intersection between the parabola from q and the current lowest parabola
				s = ((f[q] + q*q) - static_cast<double>(f[v[k]] + v[k]*v[k])) / (2*q - static_cast<double>(2*v[k]));
			} while (s <= z[k]);
			++k;
			v[k] = q;
			z[k] = s;
			z[k+1] = std::numeric_limits<double>::max();
		}
		// fill in values of distance transform
		k = static_cast<dope::SizeType>(0);
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < f.sizeAt(0); ++q) {
			while(z[k+1] < static_cast<double>(q))
				++k;
			D[q] = f[v[k]] + (q - static_cast<Scalar>(v[k]))*(q - static_cast<Scalar>(v[k]));
			Ipost[q] = Ipre[v[k]];
		}
		// delete allocated memory
		delete[] z;
		delete[] v;
	}



	template < typename Scalar, dope::SizeType DIM >
	inline void DistanceTransform::element_wiseSquareRoot(dope::DopeVector<Scalar, DIM> &m)
	{
		dope::DopeVector<Scalar, DIM-1> mm;
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < m.sizeAt(0); ++q) {
			m.at(q, mm);
			element_wiseSquareRoot(mm);
		}
	}



	template < typename Scalar >
	inline void DistanceTransform::element_wiseSquareRoot(dope::DopeVector<Scalar, 1> &m)
	{
		for (dope::SizeType q = static_cast<dope::SizeType>(0); q < m.sizeAt(0); ++q)
			m[q] = static_cast<Scalar>(std::sqrt(m[q]));
	}

}
