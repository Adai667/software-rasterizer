import React, { useState } from 'react';


function Model({ value, onModelClick }) {
  return (
    <button className="model" onClick={onModelClick}>
      {value}
    </button>
  );
}

const App = () => {
  const [imageSrc, setImageSrc] = useState('');
  const [sliderValue, setSliderValue] = useState(0);
  const [currentObject, setCurrentObject] = useState("Texture");


  const handleSliderChange = async (event) => {
    const value = event.target.value;
    setSliderValue(value); 
    document.querySelector('.slider').style.background = `linear-gradient(to right, white, white ${value}%, aquamarine` 
    
    try {
      const response = await fetch(`http://ec2-18-188-151-62.us-east-2.compute.amazonaws.com:8080/image/`+ currentObject + '/' + value);
      
      const blob = await response.blob();

      const imageUrl = URL.createObjectURL(blob);
      
      setImageSrc(imageUrl);
    } catch (error) {
      console.error('Failed to obtain image：', error);
    }
    
  };

  function handleClick(name) {
    setCurrentObject(name);
  }

  return (
    <div>
    <div class="container">
      <div class="left-section">
        <div>
          <img src={imageSrc} alt="object" />
        </div>
        
      </div>
      <div class="right-section">
        <Model value={"Classic"} onModelClick={() => handleClick("Texture")} />
        <Model value={"Cyberpunk Futurism"} onModelClick={() => handleClick("Bump")} />
        <Model value={"Metallic Glamour"} onModelClick={() => handleClick("Displacement")} />
      </div>
      
    </div>
    <div class="slider-outer">
    <input
          type="range"
          min="0"
          max="99"
          value={sliderValue}
          onChange={handleSliderChange}
          class="slider"
        />
      </div>
    </div>

  );
};

export default App;
