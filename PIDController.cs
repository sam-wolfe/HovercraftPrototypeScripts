namespace DefaultNamespace {

    class PIDController {

        public float proportionalGain;
        public float integralGain;
        public float derivitiveGain;

        private float _errorLast = 0f;

        public float Update(float dt, float currentValue, float targetValue) {
            float error = targetValue - currentValue;

            // Calculate P term
            float P = proportionalGain * error;
            
            // Calculate D term
            float errorRateOfChange = (error - _errorLast) / dt;
            _errorLast = error;

            float D = derivitiveGain * errorRateOfChange;

            return P + D;
        }

    }

}