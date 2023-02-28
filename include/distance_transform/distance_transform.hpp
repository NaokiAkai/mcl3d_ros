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

#ifndef distance_transform_hpp
#define distance_transform_hpp

#include <thread>
#include <DopeVector/Grid.hpp>

namespace dt {

	/// The DistanceTransform class provides static method to compute a distance field over any multi-dimensional regularly sampled function.
	/// The dimension is fixed at compile time.
	/// It is also possible to compute the index of the nearest minimum of each sample.
	class DistanceTransform {
	public:
		/**
		 *    @brief Compute the L2-norm distance field D of a DIM-dimensional sampled function f. D gets the distance from the local minima of f.
		 *    @param f              A DIM-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
		 *    @param nThreads       The number of threads for parallel computation. If <= 1, the computation will be sequential.
		 *    @note Arrays f and D can also be the same.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceTransformL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const bool squared = false, const std::size_t nThreads = std::thread::hardware_concurrency());

		/**
		 *    @brief Compute the L2-norm distance field D of a 1-dimensional sampled function f. D gets the distance from the local minima of f.
		 *    @param f              A 1-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
		 *    @param nThreads       The number of threads for parallel computation. Actually NOT used, since it's not easy to run a single row computation in parallel.
		 *    @note Arrays f and D can also be the same.
		 */
		template < typename Scalar >
		inline static void distanceTransformL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, const bool squared = false, const std::size_t nThreads = std::thread::hardware_concurrency());

		/**
		 *    @brief Compute the L2-norm distance field D of a DIM-dimensional sampled function f. D gets the distance from the local minima of f.
		 *           Compute also the (index of the) nearest local minimum of each sample.
		 *    @param f              A DIM-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param I              Resulting array containing the (index of the) local minimum for each sample.
		 *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
		 *    @param nThreads       The number of threads for parallel computation. If <= 1, the computation will be sequential.
		 *    @note Arrays f and D can also be the same. I should be first initialized.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceTransformL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, dope::DopeVector<dope::SizeType, DIM> &I, const bool squared = false, const std::size_t nThreads = std::thread::hardware_concurrency());

		/**
		 *    @brief Compute the L2-norm distance field D of a 1-dimensional sampled function f. D gets the distance from the local minima of f.
		 *           Compute also the (index of the) nearest local minimum of each sample.
		 *    @param f              A 1-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param I              Resulting array containing the (index of the) local minimum for each sample.
		 *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
		 *    @param nThreads       The number of threads for parallel computation. Actually NOT used, since it's not easy to run a single row computation in parallel.
		 *    @note Arrays f and D can also be the same. I sould be first initialized.
		 */
		template < typename Scalar >
		inline static void distanceTransformL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, dope::DopeVector<dope::SizeType, 1> &I, const bool squared = false, const std::size_t nThreads = std::thread::hardware_concurrency());

		/**
		 *    @brief Set up the initial indices of a DIM-dimensional sampled function.
		 *    @param I              Resulting array containing the initial index for each sample.
		 */
		template < dope::SizeType DIM >
		inline static void initializeIndices(dope::DopeVector<dope::SizeType, DIM> &I);

		/**
		 *    @brief Set up the initial indices of a 1-dimensional sampled function.
		 *    @param I              Resulting array containing the initial index for each sample.
		 */
		inline static void initializeIndices(dope::DopeVector<dope::SizeType, 1> &I);



	private:
		/**
		 *    @brief The loop iteration process that can be executed sequentially and concurrently as well.
		 *    @param f             A DIM-dimensional, regularly sampled function (a window, in multi-threading).
		 *    @param D             The resulting distance field of f (a window, in multi-threading).
		 *    @param d             The dimension where to slice.
		 *    @param order         The order in which to permute the slices.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceL2Helper(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D);

		/**
		 *    @brief The actual distance field computation is done by recursive calls of this method, in lower dimenional sub-functions.
		 *    @param f              A DIM-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D);

		/**
		 *    @brief The actual distance field computation as in the "Distance Transforms of Sampled Functions" paper, performed in a mono-dimensional function.
		 *    @param f              A 1-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 */
		template < typename Scalar >
		inline static void distanceL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D);

		/**
		 *    @brief The loop iteration process that can be executed sequentially and concurrently as well.
		 *    @param f             A DIM-dimensional, regularly sampled function (a window, in multi-threading).
		 *    @param D             The resulting distance field of f (a window, in multi-threading).
		 *    @param Ipre          Array containing the initial inedx of local minimum for each sample.
		 *    @param Ipost         Array containing the resulting index of local minimum for each sample.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceL2Helper(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const dope::DopeVector<dope::SizeType, DIM> &Ipre, dope::DopeVector<dope::SizeType, DIM> &Ipost);

		/**
		 *    @brief The actual distance field computation is done by recursive calls of this method, in lower dimenional sub-functions.
		 *    @param f              A DIM-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param I              Resulting array containing the index of the local minimum for each sample.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void distanceL2(const dope::DopeVector<Scalar, DIM> &f, dope::DopeVector<Scalar, DIM> &D, const dope::DopeVector<dope::SizeType, DIM> &Ipre, dope::DopeVector<dope::SizeType, DIM> &Ipost);

		/**
		 *    @brief The actual distance field computation as in the "Distance Transforms of Sampled Functions" paper, performed in a mono-dimensional function.
		 *    @param f              A 1-dimensional, regularly sampled function.
		 *    @param D              The resulting distance field of f.
		 *    @param I              Resulting array containing the (index of the) local minimum for each sample.
		 */
		template < typename Scalar >
		inline static void distanceL2(const dope::DopeVector<Scalar, 1> &f, dope::DopeVector<Scalar, 1> &D, const dope::DopeVector<dope::SizeType, 1> &Ipre, dope::DopeVector<dope::SizeType, 1> &Ipost);

	public:
		/**
		 *    @brief Compute the square root of each individual element of a DIM-dimensional array.
		 *    @param m              A DIM-dimensioanl array whose element have to be square rooted.
		 */
		template < typename Scalar, dope::SizeType DIM >
		inline static void element_wiseSquareRoot(dope::DopeVector<Scalar, DIM> &m);

		/**
		 *    @brief Compute the square root of each individual element of a 1-dimensional array.
		 *    @param m              A 1-dimensioanl array whose element have to be square rooted.
		 */
		template < typename Scalar >
		inline static void element_wiseSquareRoot(dope::DopeVector<Scalar, 1> &m);

	};

}

#include <distance_transform/inlines/distance_transform.inl>

#endif /* distance_transform_hpp */
