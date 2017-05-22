#include "twiddle.h"
#include "cfg.h"
#include <fstream>      // std::ofstream
#include <cmath>
#include <assert.h>     /* assert */
#include <numeric>      // accumulate
#include <iostream>

using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init( const std::vector<double>& params, 
                    const std::vector<double>& dParams,
			        const unsigned int         twiddleUpdateParam, 
		            const double               bestErr, 
			        const TwiddleMode          mode                ) 
{
  m_params             = params;
  m_deltaParams        = dParams;
                      
  m_err                = 0.0;
  
  m_twiddleUpdateParam = twiddleUpdateParam; 
  m_bestError          = bestErr;
  
  m_twiddleMode        = mode;

  m_cycleCounter       = 0U;
}

void Twiddle::UpdateError(const double cte) 
{
  ++m_cycleCounter;
  
  if (m_cycleCounter > INIT_PHASE_LENGTH)
  {
	  m_err += std::pow(cte, 2.0);
  }
}

bool Twiddle::EstimateTwiddle() 
{
	std::cout << m_cycleCounter << "\t" << ACCU_PHASE_LENGTH << std::endl;
	if (m_cycleCounter < ACCU_PHASE_LENGTH)
	{
	  return false;
	}
	
    const double delta = std::accumulate(m_params.begin(), m_params.end(), 0);
	if (delta > TOL)
	{
		switch (m_twiddleMode)
		{
		case TWIDDLE_INIT:
		  {
			TwiddleReInit(m_err, 0.0, 1.0, 1.0, m_twiddleUpdateParam, TWIDDLE_TRIAL);
		  }
		  break;
		case TWIDDLE_TRIAL:
		  {
		    if (m_err < m_bestError)
			{ 
		      const unsigned int nextUpdateParam = (m_twiddleUpdateParam+1)%m_params.size();
		      TwiddleReInit(m_err, 1.0, 1.1, 0.0, nextUpdateParam, TWIDDLE_TRIAL);
			}
			else
			{
			  TwiddleReInit(m_bestError, 0.0, 1.0, -2.0, m_twiddleUpdateParam, TWIDDLE_CHANGE_SIGN);
			}
		  }
		  break;
		case TWIDDLE_CHANGE_SIGN:
		  {
			const unsigned int nextUpdateParam = (m_twiddleUpdateParam+1)%m_params.size();
		    if (m_err < m_bestError)
			{ 
		      TwiddleReInit(m_err, 1.0, 1.1, 0.0, nextUpdateParam, TWIDDLE_TRIAL);
			}
			else
			{
		      TwiddleReInit(m_bestError, 1.0, 0.9, 1.0, nextUpdateParam, TWIDDLE_TRIAL);
			}
		  }
		  break;
		default:
		  {
		    assert(0 && "Unknown twiddle mode");
		  }
		  break;			
		}
		
		return true;
	}
	return false;
}

void Twiddle::TwiddleReInit(const double best_err,                const double   pChangeValFactorNext, const double dpIncreaseFactor, 
                            const double pChangeValFactorCurrent, const unsigned int nextUpdateParam,  const TwiddleMode nextMode)
{  
  // Dump last parameters
  std::ofstream ofs;
  ofs.open("params.txt", std::ofstream::out | std::ofstream::app);
  
  for (auto const& c : m_params)
    ofs << c << "\t";
	
  for (auto const& c : m_deltaParams)
    ofs << c << "\t";
	
  ofs << m_err << "\t" << m_twiddleUpdateParam << "\t" << m_twiddleMode << "\t" << nextUpdateParam << "\n";
  ofs.close();
  
  // Change parameter for current coordinate
  m_params[m_twiddleUpdateParam]  += (m_deltaParams[m_twiddleUpdateParam]*pChangeValFactorCurrent);

  // Change step size for this coordinate
  m_deltaParams[m_twiddleUpdateParam] *= dpIncreaseFactor;  
  
  // Change parameter for next run
  m_params[nextUpdateParam]       += (m_deltaParams[nextUpdateParam]*pChangeValFactorNext);
  
  // Set new filter parameters
  Init(m_params, m_deltaParams, nextUpdateParam, best_err, nextMode);
}

