export interface CheckboxProps {
  text: string;
  onChange: () => void;
}

export const Checkbox = ({ text, onChange }: CheckboxProps) => {
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