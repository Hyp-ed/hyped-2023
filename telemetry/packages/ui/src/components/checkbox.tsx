export interface ButtonProps {
  text: string;
  onChange: () => void;
}

const Checkbox = ({ text, onChange }: ButtonProps) => {
  const id = `checkbox-${text.replace(' ', '-').toLowerCase()}`;

  return (
    <div className="flex gap-4 cursor-pointer max-w-max">
      <input
        className="cursor-pointer"
        type="checkbox"
        id={id}
        onChange={onChange}
      />
      <label htmlFor={id} className="cursor-pointer text-lg">
        {text}
      </label>
    </div>
  );
};

export default Checkbox;
