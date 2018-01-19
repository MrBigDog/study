#ifndef GWGEOLOGICALTUIL_RANDOM_H
#define GWGEOLOGICALTUIL_RANDOM_H 1

#include <gwGeologicalUtil/Export.h>
#include <gwGeologicalUtil/Common.h>

namespace gwUtil
{
	/**
	 * Psuedo random number generator. Results are guaranteed to be consistent across
	 * machines given the same seed value.
	 */
	class GWGEOLOGICALUTIL_EXPORT Random
	{
	public:
		enum Method
		{
			METHOD_FAST        // not great, but super-fast and cheap.
		};

	public:
		/**
		 * Constructs a new random number generator. It is seeded with
		 * the system clock.
		 * @param method RNG method to use
		 */
		Random(Method method = METHOD_FAST);

		/**
		 * Constucts a new random number generator with a user-specified seed.
		 * The results are guaranteed to be globally consistent.
		 * @param seed   RNG seed value
		 * @param method RNG method to use
		 */
		Random(unsigned seed, Method method = METHOD_FAST);

		/**
		 * Copy constructor.
		 */
		Random(const Random& rhs);

		/** dtor */
		virtual ~Random() { }

		/**
		 * Seeds the PRNG with a new seed.
		 */
		void seed(unsigned s);

		/**
		 * Resets the PRNG to its initial state (initial seed).
		 */
		void reset();

		/**
		 * Gets the next random number as an unsigned int.
		 * @param mod Modulus value. The result will fall within the range [0..mod)
		 */
		unsigned next(unsigned mod);

		/**
		 * Gets the next random number as a double in the range [0..1].
		 */
		double next();

	private:
		Method   _method;
		unsigned _seed;
		unsigned _next;
	};
}

#endif // GWGEOLOGICALTUIL_RANDOM_H
