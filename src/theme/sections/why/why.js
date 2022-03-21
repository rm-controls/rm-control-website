import React from "react";
import Container from "./../../components/container/container";
import WhyContext from "../../../../current_docs/overview/why_rm-controls.md";

export default function Why() {
  const component = "shift-why";

  return (
    <div className={component}>
      <Container componentClass={component} size={"medium"}>
        <WhyContext />
      </Container>
    </div>
  );
}
