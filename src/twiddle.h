#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <assert.h>     /* assert */

enum TwiddleMode
{
	TWIDDLE_INIT,
	TWIDDLE_TRIAL,
	TWIDDLE_CHANGE_SIGN
};

class Twiddle {
public:

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  /*
  * Initialize Twiddle.
  */
  void Init(const std::vector<double>& params, 
            const std::vector<double>& dParams,
			const unsigned int         twiddleUpdateParam, 
		    const double               bestErr, 
			const TwiddleMode          mode  );

  /*
  * Update the Twiddle error variables given cross track error.
  */
  void UpdateError(const double cte);
  
  /*
  * Do twiddle parameter update. Return if one update cycle was done or not.
  */
  bool EstimateTwiddle();
  
  // Getter for parameters
  double getParam(const unsigned int idx) const {
	assert(idx < m_params.size()); 
	return m_params[idx]; 
  }
  
  private:
  
  /*
  * Set twiddle parameters for next cycle.
  */
  void TwiddleReInit( const double       best_err, 
                      const double       pChangeValFactorNext, 
					  const double       dpIncreaseFactor, 
				      const double       pChangeValFactorCurrent, 
					  const unsigned int nextUpdateParam, 
					  const TwiddleMode  nextMode);
					  
  /*
  * Parameters
  */ 
  std::vector<double> m_params;
  
  /*
  * Twiddle Update parameters.
  */
  std::vector<double> m_deltaParams;

  /*
  * Algo cycle counter.
  */
  unsigned int m_cycleCounter;
  
  /*
  * Accumulated squared cte for Twiddle.
  */
  double m_err;
  double m_bestError;
  
  TwiddleMode m_twiddleMode;
  
  unsigned int m_twiddleUpdateParam;
};

#endif /* TWIDDLE_H */
