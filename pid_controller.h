/* Created and copyrighted by Zachary J. Fields. Offered as open source under the MIT License (MIT). */

//! \brief A (P)roportional, (I)ntegral, (D)erivative Controller
//! \details The controller should be able to control any process with a
//! measureable value, a known ideal value and an input to the
//! process that will affect the measured value.
//! \see { <a href="https://en.wikipedia.org/wiki/PID_controller">Wikipedia: PID Controller</a> }
class PidController {
  public:

    //! \brief Type associated with the callback for
    //! sampling the process variable in the feedback loop
    typedef float(*process_variable_callback_t)(void);

    //! \brief Constructor
    //! \param[in] sampleProcessVariable_ A user supplied callback to update
    //! the value of the process variable
    //! \param[in] gain_proportional_ The gain applied to the proportional
    //! result [default value: 1.0]
    //! \param[in] gain_integral_ The gain applied to the integral
    //! result [default value: 0.5]
    //! \param[in] gain_derivative_ The gain applied to the derivative
    //! result [default value: 0.0]
    PidController(
        const process_variable_callback_t sampleProcessVariable_,
        const float gain_proportional_ = 1.0f,
        const float gain_integral_ = 0.5f,
        const float gain_derivative_ = 0.0f
    ) :
        _error_accumulated(0.0f),
        _gain_derivative(gain_derivative_),
        _gain_integral(gain_integral_),
        _gain_proportional(gain_proportional_),
        _process_variable(0.0f),
        _process_variable_last(0.0f),
        _sampleProcessVariable(sampleProcessVariable_),
        _set_point(0.0f)
    {
        // Set the system into a stable state
        if ( _sampleProcessVariable ) {
            _process_variable = _sampleProcessVariable();
            _process_variable_last = _process_variable;
            _set_point = _process_variable;
        }
    }

    //! \brief The output of the controller
    //! \details The output value is a signed, unitless, relative
    //! value that indicates how the actuator (i.e. motor, relay,
    //! etc...) should influence the feedback loop. This PID
    //! implementation is designed to have this function called
    //! at a non-realtime, but relatively consistent, time interval.
    inline
    float
    controlVariable (
        void
    ) {
        if ( _sampleProcessVariable ) {
            _process_variable_last = _process_variable;
            _process_variable = _sampleProcessVariable();
        }
        return (
            (_gain_proportional * inputProportional())
          + (_gain_integral * inputIntegral())
          + (_gain_derivative * inputDerivative())
        );
    }

    //! \brief Set the target or desired value of the process variable
    //! \details The PID controller will attempt to move influence the
    //! process variable, via the controlVariable, to reach this value.
    //! \note When the controller is constructed this value will be
    //! set to the current value of the process variable, by calling
    //! the user supplied callback.
    inline
    void
    setPoint (
        const float set_point_
    ) {
        _set_point = set_point_;
    }

    //! \brief The target or desired value of the process variable
    inline
    float
    setPoint (
        void
    ) const {
        return _set_point;
    }

  private:
    //! \brief The difference between the current value and the desired value
    inline
    float
    error (
        void
    ) const {
        return (_set_point - _process_variable);
    }

    //! \brief Adjustment made by considering the rate of change of the error
    //! \details This function returns the inverse of the slope to dampen overshoot.
    //! \note The calculation simplifies to subtraction, because the denominator of the
    //! slope - the time interval between samples - is one (unit).
    inline
    float
    inputDerivative (
        void
    ) const {
        return (_process_variable_last - _process_variable);
    }

    //! \brief Adjustment made by considering the accumulated error over time
    //! \details The integral term is proportional to both the
    //! magnitude of the error and the duration of the error.
    //! An alternative formulation of the integral action, is the
    //! proportional-summation-difference used in discrete-time systems
    inline
    float
    inputIntegral (
        void
    ) {
        float current_error = error();

        // Test reset conditions for accumulated error
        if ( !current_error
        || (0 > current_error && 0 < _error_accumulated)
        || (0 < current_error && 0 > _error_accumulated) ) {
            _error_accumulated = 0.0f;
        }

        return (_error_accumulated += current_error);
    }

    //! \brief Adjustment made in proportion to the current
    //! error value
    inline
    float
    inputProportional (
        void
    ) const {
        return error();
    }

    //! \brief The area under the curve created by the
    //! oscillation of the PID controller
    float _error_accumulated;

    //! \brief The gain associated to the derivative term
    const float _gain_derivative;

    //! \brief The gain associated to the integral term
    //! \note The gain should be relative to the ratio of the
    //! sampling rate to the rate of change of the process
    //! variable in the feedback loop, as this will be reflected
    //! in the accumulated error.
    const float _gain_integral;

    //! \brief The gain associated to the proportional term
    //! \note Tuning theory and industrial practice indicate
    //! that the proportional term should contribute the bulk
    //! of the output change.
    const float _gain_proportional;

    //! \brief The measured value in the feedback loop
    //! \note This value is set by the user supplied
    //! callback function `_sampleProcessVariable`
    float _process_variable;

    //! \brief The value of the prior measurement of the
    //! feedback loop
    //! \details This value, when subtracted from the current
    //! value of the process variable, is used to calculate
    //! the rate of change (a.k.a. slope) of the curve of
    //! historical values in the feedback loop
    float _process_variable_last;

    //! \brief A user supplied callback function to update
    //! the value of the process variable (feedback)
    const process_variable_callback_t _sampleProcessVariable;

    //! \brief The desired value
    float _set_point;
};

/* Created and copyrighted by Zachary J. Fields. Offered as open source under the MIT License (MIT). */
